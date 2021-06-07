use logos::Logos;
use std::{ffi::OsString, io::Read};

use crate::scene::token::Token;

pub struct Lexer {
    root_file: OsString,
    root_dir: std::path::PathBuf,
}

impl Lexer {
    pub fn from_file(path: &str) -> Result<Lexer, &str> {
        let path = std::path::Path::new(path);

        match path.parent() {
            None => Err("no parent directory available"),
            Some(parent) => {
                let root_file = path.file_name().unwrap().to_owned();
                Ok(Lexer {
                    root_file,
                    root_dir: std::path::PathBuf::from(parent),
                })
            }
        }
    }

    pub fn read_tokens(&self) -> Vec<Token> {
        let mut dir = self.root_dir.clone();
        dir.push(self.root_file.clone());
        let mut in_file = std::fs::File::open(dir).expect("open file failed");
        let mut content = String::new();
        in_file
            .read_to_string(&mut content)
            .expect("reading file failed");

        let mut tokenizer = Token::lexer(&content);

        // tokenizer.collect::<Vec<_>>()
        let mut tokens = Vec::<Token>::new();
        while let Some(t) = tokenizer.next() {
            if t != Token::Include {
                tokens.push(t);
            } else {
                let next_t = tokenizer
                    .next()
                    .expect("should have a file name after include");
                if let Token::QuotedString(file_name) = next_t {
                    let mut included_file_path = self.root_dir.clone();
                    included_file_path.push(file_name);
                    let nested_lexer = Lexer::from_file(included_file_path.to_str().unwrap())
                        .expect("Can't read included file");
                    tokens.append(&mut nested_lexer.read_tokens());
                } else {
                    panic!("should have a file name after include, but instead {:?}", next_t);
                }
            }
        }
        tokens
    }
}
