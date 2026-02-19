use proc_macro::TokenStream;
use quote::quote;
use syn::{ItemEnum, parse_macro_input};

#[proc_macro_derive(CommandId)]
pub fn derive_command_id(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as ItemEnum);

    let mut structs = Vec::new();

    for (idx, variant) in input.variants.iter().enumerate() {
        let variant_name = &variant.ident;
        structs.push(quote! {
            pub struct #variant_name;
            impl CommandId for #variant_name {
                const COMMAND_ID: u8 = #idx as u8;
            }
        });
    }

    TokenStream::from(quote! {
        #(#structs)*
    })
}
