use proc_macro::TokenStream;
use syn::parse::{Parse, ParseStream};
use syn::{Token, LitStr};
use quote::quote;

mod c_parser;

/// Generates Rust RPC client methods from C RPC wrapper functions.
///
/// # Example
/// ```rust,ignore
/// rpc_from_c!(cmd = "BtEnableRpcCmd", sig = "bt_enable(bt_ready_cb_t cb)");
/// ```
#[proc_macro]
pub fn rpc_from_c(input: TokenStream) -> TokenStream {
    let args = syn::parse_macro_input!(input as RpcFromCArgs);

    // Parse the C function signature
    let c_function = match c_parser::parse_c_signature(&args.sig) {
        Ok(func) => func,
        Err(e) => {
            return syn::Error::new(
                proc_macro2::Span::call_site(),
                format!("Failed to parse C signature '{}': {}", args.sig, e)
            ).to_compile_error().into();
        }
    };

    // Generate the RPC method
    let fn_name = syn::Ident::new(&c_function.name, proc_macro2::Span::call_site());
    let command_id_ident = syn::Ident::new(&args.cmd, proc_macro2::Span::call_site());
    
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
        quote! {
            // TODO: add a no-response helper on RpcClient when we
            // need to generate wrappers for void-returning commands.
            // For now, this branch is unused.
            let _ = packet;
            Ok(())
        }
    } else {
        quote! {
            let result = client.send_command_and_get_i32(packet).await?;
            Ok(result)
        }
    };
    
    let generated = quote! {
        pub async fn #fn_name<T: nrf_rpc::AsyncTransport>(
            client: &mut nrf_rpc::RpcClient<T>,
            #(#fn_params),*
        ) -> Result<#return_type, nrf_rpc::RpcError> {
            // Build CBOR payload
            let mut buffer = [0u8; 256];
            let mut builder = nrf_rpc::cbor_encoding::CborPayloadBuilder::new(&mut buffer);
            
            // Encode scratchpad size (always 0 for now)
            builder = builder.encode_uint_64(0)?;
            
            // Encode parameters
            #(#param_encodings)*
            
            let payload = builder.build()?;
            
            // Build RPC packet
            let packet = nrf_rpc::packet::NrfRpcPacket::<nrf_rpc::packet::Command>::new(
                nrf_rpc::packet::SrcContextId::try_from(client.context_id())
                    .map_err(|_| nrf_rpc::RpcError::Transport)?,
                nrf_rpc::packet::DestContextId::try_from(0xFF)
                    .map_err(|_| nrf_rpc::RpcError::Transport)?,
                nrf_rpc::packet::CommandId::try_from(nrf_rpc::ble::BleClientCommandId::#command_id_ident as u8)
                    .map_err(|_| nrf_rpc::RpcError::Transport)?,
                nrf_rpc::packet::SrcGroupId::try_from(client.bt_rpc_group_id())
                    .map_err(|_| nrf_rpc::RpcError::Transport)?,
                nrf_rpc::packet::DstGroupId::try_from(client.bt_rpc_group_id())
                    .map_err(|_| nrf_rpc::RpcError::Transport)?,
                payload,
            );
            
            // Send command and handle response
            #send_logic
        }
    };

    generated.into()
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
        CType::Struct(_name) => {
            // For struct types, encode as callback slot or custom encoding
            (
                quote! { u64 },
                quote! {
                    builder = builder.encode_uint_64(#param_name)?;
                },
            )
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

