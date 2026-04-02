//! Simple C function signature parser for RPC code generation.

/// Check if a string is a valid C identifier
fn is_valid_identifier(s: &str) -> bool {
    if s.is_empty() {
        return false;
    }
    
    let mut chars = s.chars();
    let first = chars.next().unwrap();
    
    // First character must be letter or underscore
    if !first.is_ascii_alphabetic() && first != '_' {
        return false;
    }
    
    // Rest can be letters, digits, or underscores
    chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct CFunctionInfo {
    pub name: String,
    pub parameters: Vec<CParameter>,
    pub return_type: CType,
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct CParameter {
    pub name: String,
    pub c_type: CType,
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub enum CType {
    Void,
    Int,
    UInt8,
    UInt16,
    UInt32,
    SizeT,
    Bool,
    Struct(String),
    Pointer(Box<CType>),
    ConstPointer(Box<CType>),
}

/// Parses a C function signature string.
/// Format: "function_name(type1 param1, type2 param2)"
/// Example: "bt_enable(bt_ready_cb_t cb)"
pub fn parse_c_signature(signature: &str) -> Result<CFunctionInfo, String> {
    let signature = signature.trim();

    let open_paren = signature
        .find('(')
        .ok_or_else(|| format!("No opening parenthesis in: {}", signature))?;
    let close_paren = signature
        .rfind(')')
        .ok_or_else(|| format!("No closing parenthesis in: {}", signature))?;

    if close_paren < open_paren {
        return Err("Closing parenthesis before opening".to_string());
    }

    let before_paren = signature[..open_paren].trim();
    let params_str = signature[open_paren + 1..close_paren].trim();

    // Parse function name (last token before parenthesis)
    let tokens: Vec<&str> = before_paren.split_whitespace().collect();
    let function_name = tokens
        .last()
        .ok_or_else(|| "No function name found".to_string())?
        .to_string();

    // Parse return type (everything before function name, or void if empty)
    let return_type = if tokens.len() > 1 {
        parse_c_type(&tokens[..tokens.len() - 1].join(" "))?
    } else {
        CType::Void
    };

    let parameters = parse_parameters_from_string(params_str)?;

    Ok(CFunctionInfo {
        name: function_name,
        parameters,
        return_type,
    })
}

fn parse_c_type(type_str: &str) -> Result<CType, String> {
    let type_str = type_str.trim();

    if type_str == "void" {
        return Ok(CType::Void);
    }

    let is_const = type_str.starts_with("const ");
    let type_str_no_const = if is_const { &type_str[6..] } else { type_str };

    let pointer_count = type_str_no_const.matches('*').count();
    let base_type_str = type_str_no_const.replace('*', "").trim().to_string();

    let mut base_type = match base_type_str.as_str() {
        "void" => CType::Void,
        "int" => CType::Int,
        "uint8_t" => CType::UInt8,
        "uint16_t" => CType::UInt16,
        "uint32_t" => CType::UInt32,
        "size_t" => CType::SizeT,
        "bool" | "_Bool" => CType::Bool,
        _ => {
            if base_type_str.starts_with("struct ") {
                CType::Struct(base_type_str[7..].trim().to_string())
            } else {
                CType::Struct(base_type_str)
            }
        }
    };

    for _ in 0..pointer_count {
        base_type = if is_const {
            CType::ConstPointer(Box::new(base_type))
        } else {
            CType::Pointer(Box::new(base_type))
        };
    }

    Ok(base_type)
}

fn parse_parameters_from_string(params_str: &str) -> Result<Vec<CParameter>, String> {
    if params_str.is_empty() || params_str == "void" {
        return Ok(Vec::new());
    }

    let param_strings = split_parameters(params_str);
    let mut parameters = Vec::new();

    for (idx, param_str) in param_strings.iter().enumerate() {
        let param_str = param_str.trim();
        let tokens: Vec<&str> = param_str.split_whitespace().collect();

        if tokens.is_empty() {
            return Err(format!("Empty parameter at position {}", idx));
        }

        // For simple case like "type_name param_name", last token is param name
        // Everything before is the type
        if tokens.len() >= 2 {
            let mut name = tokens.last().unwrap().to_string();
            let mut type_tokens = tokens[..tokens.len() - 1].to_vec();
            
            // Handle pointer/asterisk attached to parameter name: "type *name"
            if name.starts_with('*') {
                // Move the * from the name to the type
                type_tokens.push("*");
                name = name[1..].to_string();
            }
            
            if !is_valid_identifier(&name) {
                return Err(format!("`{:?}` is not a valid identifier", name));
            }
            
            let type_str = type_tokens.join(" ");
            let c_type = parse_c_type(&type_str)?;
            parameters.push(CParameter { name, c_type });
        } else {
            // Single token - assume it's a type with anonymous parameter
            let name = format!("param{}", idx);
            let type_str = tokens[0];
            let c_type = parse_c_type(type_str)?;
            parameters.push(CParameter { name, c_type });
        }
    }

    Ok(parameters)
}

fn split_parameters(params_str: &str) -> Vec<String> {
    let mut result = Vec::new();
    let mut current = String::new();
    let mut paren_depth = 0;

    for ch in params_str.chars() {
        match ch {
            '(' => {
                paren_depth += 1;
                current.push(ch);
            }
            ')' => {
                paren_depth -= 1;
                current.push(ch);
            }
            ',' if paren_depth == 0 => {
                result.push(current.trim().to_string());
                current.clear();
            }
            _ => current.push(ch),
        }
    }

    if !current.trim().is_empty() {
        result.push(current.trim().to_string());
    }

    result
}
