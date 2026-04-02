use proc_macro::TokenStream;
use quote::quote;
use syn::parse::{Parse, ParseStream};
use syn::{parenthesized, punctuated::Punctuated, DeriveInput, LitStr, Token};

mod c_parser;

struct MacroInput {
    rpc_fns: Vec<RpcFromCArgs>,
}

impl Parse for MacroInput {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        // Input of form (cmd = "string", sig = "string"),
        //               (...),
        //               (cmd = "string, sig = "string)

        let rpc_fn_list: Punctuated<RpcFromCArgs, syn::token::Comma> =
            input.parse_terminated(RpcFromCArgs::parse, syn::token::Comma)?;

        let mut rpc_fns: Vec<RpcFromCArgs> = vec![];
        for item in rpc_fn_list {
            rpc_fns.push(item)
        }

        Ok(Self { rpc_fns })
    }
}

/// Generates Rust RPC client methods from C RPC wrapper functions.
///
/// # Example
/// ```rust,ignore
/// rpc_from_c!(cmd = "BtEnableRpcCmd", sig = "bt_enable(bt_ready_cb_t cb)");
/// ```
#[proc_macro_attribute]
pub fn rpc_from_c(attr: TokenStream, item: TokenStream) -> TokenStream {
    let parsed_input = syn::parse_macro_input!(attr as MacroInput);

    // We do not modify the original struct in this proc macro,
    // so add to the output before consuming/parsing the struct.
    let mut output: proc_macro2::TokenStream = item.clone().into();

    let ast: DeriveInput =
        syn::parse(item).expect("Error Parsing Provided AST of register struct.");

    let data = match &ast.data {
        syn::Data::Struct(data) => data,
        _ => panic!("Unsupported type for `rpc_from_c` macro; must be a struct."),
    };

    let mut client_arg_name: Option<syn::Ident> = None;

    // Search each field for one with type `RpcClient`.
    for field_item in data.fields.clone() {
        if let syn::Type::Path(type_path) = field_item.ty {
            for path_item in type_path.path.segments {
                if path_item.ident.to_string().contains("RpcClient") {
                    client_arg_name = field_item.ident;
                    break;
                }
            }
        }
    }

    let client_arg_name = client_arg_name
        .expect("Error - `rpc_from_c` macro requires the struct contain an `RpcClient` member.");

    let mut generated_funcs: Vec<_> = vec![];

    for fn_itm in parsed_input.rpc_fns {
        generated_funcs.push(fn_itm.generate_fn(&client_arg_name));
    }

    let (impl_generics, ty_generics, where_clause) = ast.generics.split_for_impl();
    let struct_name = ast.ident;

    output.extend(quote! {
        impl #impl_generics #struct_name #ty_generics #where_clause {
            #(#generated_funcs)*
        }
    });

    output.into()
}

/// Maps C types to Rust types and generates CBOR encoding code
fn map_c_type_to_rust_and_encoding(
    c_type: &c_parser::CType,
    param_name: &syn::Ident,
) -> (proc_macro2::TokenStream, proc_macro2::TokenStream) {
    use c_parser::CType;

    match c_type {
        CType::Int => (
            quote! { i32 },
            quote! {
                builder = builder.encode_int_32(#param_name)?;
            },
        ),
        CType::UInt8 => (
            quote! { u8 },
            quote! {
                builder = builder.encode_uint_8(#param_name)?;
            },
        ),
        CType::UInt16 => (
            quote! { u16 },
            quote! {
                builder = builder.encode_uint_16(#param_name)?;
            },
        ),
        CType::UInt32 => (
            quote! { u32 },
            quote! {
                builder = builder.encode_uint_32(#param_name)?;
            },
        ),
        CType::SizeT => (
            quote! { usize },
            quote! {
                builder = builder.encode_uint_64(#param_name as u64)?;
            },
        ),
        CType::Bool => (
            quote! { bool },
            quote! {
                builder = builder.encode_uint_8(if #param_name { 1 } else { 0 })?;
            },
        ),
        CType::Pointer(_) | CType::ConstPointer(_) => {
            // For pointers, encode as callback slot (uint64)
            (
                quote! { u64 },
                quote! {
                    builder = builder.encode_uint_64(#param_name)?;
                },
            )
        }
        CType::Struct(name) => {
            // Check if this is a callback type (ends with _cb_t or _callback)
            if name.ends_with("_cb_t") || name.ends_with("_callback") {
                // For callback types, use Option<u32> where:
                // - None encodes as CBOR null
                // - Some(slot) encodes as CBOR int32 (callback slot ID)
                (
                    quote! { Option<u32> },
                    quote! {
                        builder = match #param_name {
                            None => builder.cbor_null()?,
                            Some(slot) => builder.encode_int_32(slot as i32)?,
                        };
                    },
                )
            } else {
                // For other struct types, encode as u64
                (
                    quote! { u64 },
                    quote! {
                        builder = builder.encode_uint_64(#param_name)?;
                    },
                )
            }
        }
        CType::Void => (quote! { () }, quote! {}),
    }
}

/// Parse arguments: client = ..., cmd = "...", sig = "..."
struct RpcFromCArgs {
    cmd: String,
    sig: String,
}

impl Parse for RpcFromCArgs {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        // Confirm first/last token are parenthesis.
        // (the error for Parenthesized!() is a bit lacking)
        if !input.peek(syn::token::Paren) {
            return Err(syn::Error::new(
                input.span(),
                "Error - expected `(` to begin
            defined rpc c function input. Each fn to be generated should be of the form
            (cmd = \"NAME\", sig = \"FN SIGNATURE\"",
            ));
        }

        let inner;
        let _ = parenthesized!(inner in input);

        let input = inner;

        let mut cmd: Option<String> = None;
        let mut sig: Option<String> = None;
        while !input.is_empty() {
            let ident: syn::Ident = input.parse()?;
            let _: Token![=] = input.parse()?;

            if ident == "cmd" {
                let value: LitStr = input.parse()?;
                cmd = Some(value.value());
            } else if ident == "sig" {
                let value: LitStr = input.parse()?;
                sig = Some(value.value());
            } else {
                return Err(syn::Error::new_spanned(ident, "expected 'cmd' or 'sig'"));
            }

            if !input.is_empty() {
                let _: Token![,] = input.parse()?;
            }
        }

        Ok(RpcFromCArgs {
            cmd: cmd.ok_or_else(|| input.error("missing 'cmd' parameter"))?,
            sig: sig.ok_or_else(|| input.error("missing 'sig' parameter"))?,
        })
    }
}

impl RpcFromCArgs {
    /// Perform code generation for a parsed C RPC function.
    fn generate_fn(&self, client_arg_name: &syn::Ident) -> proc_macro2::TokenStream {
        // Parse the C function signature
        let c_function = match c_parser::parse_c_signature(&self.sig) {
            Ok(func) => func,
            Err(e) => {
                return syn::Error::new(
                    proc_macro2::Span::call_site(),
                    format!("Failed to parse C signature '{}': {}", &self.sig, e),
                )
                .to_compile_error()
                .into();
            }
        };

        // Generate the RPC method
        let fn_name = syn::Ident::new(&c_function.name, proc_macro2::Span::call_site());
        let command_id_ident = syn::Ident::new(&self.cmd, proc_macro2::Span::call_site());

        // Map C return type to Rust
        let return_type = match c_function.return_type {
            c_parser::CType::Void => quote! { () },
            c_parser::CType::Int => quote! { i32 },
            _ => quote! { () },
        };

        // Generate function parameters from C signature
        let mut fn_params = Vec::new();
        let mut param_encodings = Vec::new();

        for param in &c_function.parameters {
            let param_name = syn::Ident::new(&param.name, proc_macro2::Span::call_site());
            let (rust_type, encoding) = map_c_type_to_rust_and_encoding(&param.c_type, &param_name);

            fn_params.push(quote! { #param_name: #rust_type });
            param_encodings.push(encoding);
        }

        // Determine send method based on return type. The generated API uses
        // nrf_rpc::RpcError directly so that the resulting code remains
        // no_std-compatible (no dependency on std::String or formatting).
        let send_logic = if matches!(c_function.return_type, c_parser::CType::Void) {
            todo!()
        } else {
            quote! {
                self.#client_arg_name.send_command_and_get_i32(packet).await
            }
        };

        let generated = quote! {
            pub async fn #fn_name(
                 &mut self,
                #(#fn_params),*
            ) -> Result<#return_type, nrf_rpc::RpcError> {
                // Build CBOR payload
                let mut buffer = [0u8; 256];
                let mut builder = nrf_rpc::cbor_encoding::CborPayloadBuilder::new(&mut buffer);

                // Encode parameters
                #(#param_encodings)*

                let payload = builder.build()?;

                // Build RPC packet
                let packet = nrf_rpc::packet::NrfRpcPacket::<nrf_rpc::packet::Command>::new(
                    nrf_rpc::packet::SrcContextId::try_from(self.#client_arg_name.context_id())
                        .map_err(|_| nrf_rpc::RpcError::Transport)?,
                    nrf_rpc::packet::DestContextId::try_from(0xFF)
                        .map_err(|_| nrf_rpc::RpcError::Transport)?,
                    nrf_rpc::packet::CommandId::try_from(BleClientCommandId::#command_id_ident as u8)
                        .map_err(|_| nrf_rpc::RpcError::Transport)?,
                    nrf_rpc::packet::SrcGroupId::try_from(self.#client_arg_name.bt_rpc_group_id())
                        .map_err(|_| nrf_rpc::RpcError::Transport)?,
                    nrf_rpc::packet::DstGroupId::try_from(self.#client_arg_name.bt_rpc_group_id())
                        .map_err(|_| nrf_rpc::RpcError::Transport)?,
                    payload,
                );

                // Send command and handle response
                #send_logic
            }
        };

        generated.into()
    }
}
