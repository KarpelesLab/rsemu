//! The recursive-descent parser for `.machine` files.
//!
//! Hand-written, like the lexer: zero dependencies (`ROADMAP.md` §0), and a
//! generator would not produce the error messages §5 demands anyway.
//!
//! # Shape of the grammar
//!
//! A file is a flat list of statements, and so is every block. Each statement
//! begins with a word that says what it is, which is what keeps the graph
//! scannable and what makes a wrong word a one-line diagnostic:
//!
//! ```text
//! unit      := stmt* EOF
//! stmt      := machine | param | osc | space | object | map | wire
//!            | include | template | instance | for
//! machine   := "machine" STRING block
//! param     := "param" name ("=" expr)?
//! osc       := "osc" name "=" expr freq-unit
//! space     := "space" name props
//! object    := "object" name STRING props?
//! map       := "map" name expr "size" expr "=" expr props?
//! wire      := "wire" path "->" path
//! include   := "include" STRING
//! template  := "template" name ("(" param-list ")")? block
//! instance  := "instance" name "=" name ("(" arg-list ")")?
//! for       := "for" name "in" expr (".." | "..=") expr block
//! props     := "{" (name "=" expr ","?)* "}"
//! expr      := additive
//! additive  := multiplicative (("+"|"-") multiplicative)*
//! multiplicative := unary (("*"|"/"|"%") unary)*
//! unary     := "-" unary | primary
//! primary   := NUMBER | STRING | "true" | "false" | path | call
//!            | "(" expr ")" | "[" expr,* "]" | "{" (name "=" expr ","?)* "}"
//! call      := path "(" expr,* ")"          -- "(" must touch the path
//! path      := name ("." name)*
//! name      := (IDENT | "$" IDENT | "$" "{" expr "}")+   -- parts must touch
//! ```
//!
//! # Decisions §5 does not make for us
//!
//! * **Separators are optional.** `{ width = 16, unassigned = open-bus }` and a
//!   comma-less multi-line block are both in §5, so commas are permitted and
//!   never required. Newlines are not tokens; instead no expression can extend
//!   past its line by accident, because a call's `(` must touch its callee and
//!   there is no juxtaposition operator.
//! * **`instance`** is this parser's spelling of "instantiate a template". §5
//!   requires templates "instantiated N times" but shows no syntax; a named
//!   instantiation is chosen because a template declares several objects and
//!   the instance name is the natural namespace for them.
//! * **One error, then stop.** No resynchronisation: a recovering parser
//!   invents cascades, and §5 asks for a message a person can act on.
//!
//! # Robustness
//!
//! Nesting is capped at [`MAX_DEPTH`], which bounds both parser recursion and
//! the recursion in dropping the tree, so no input — truncated, adversarial, or
//! merely enthusiastic — can overflow the stack. Nothing here indexes, slices
//! or does unchecked arithmetic on untrusted values.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::machine::ast::{
    Arg, BinOp, Expr, ForStmt, FreqUnit, IncludeStmt, InstanceStmt, MachineDecl, MapStmt, Name,
    NamePart, ObjectDecl, OscDecl, ParamDecl, Path, Property, SourceUnit, SpaceDecl, Stmt,
    TemplateDecl, TemplateParam, UnOp, WireStmt,
};
use crate::machine::diag::Diagnostic;
use crate::machine::lexer::{Token, TokenKind, tokenize};
use crate::machine::span::{SourceFile, Span, Spanned};

/// How deeply blocks and expressions may nest.
///
/// A machine description does not legitimately nest anywhere near this far; the
/// limit exists so that `((((((…))))))` is an error message rather than a stack
/// overflow. It bounds tree depth, so dropping the result is bounded too.
pub const MAX_DEPTH: u32 = 64;

/// Every word that starts a statement, in the order the error message lists
/// them.
const STATEMENT_KEYWORDS: &[&str] = &[
    "machine", "param", "osc", "space", "object", "map", "wire", "include", "template", "instance",
    "for",
];

/// Parse a whole file into an AST.
///
/// The file is not read from disk — `no_std` has no filesystem and the include
/// search path belongs to the caller (`ROADMAP.md` §5). Returns the first
/// error; see [`Diagnostic::render`](crate::machine::diag::Diagnostic::render)
/// for turning it into something a person can read.
pub fn parse(src: &SourceFile<'_>) -> Result<SourceUnit, Diagnostic> {
    let tokens = tokenize(src)?;
    let eof = Token {
        kind: TokenKind::Eof,
        span: Span::at(tokens.last().map_or(0, |t| t.span.end)),
    };
    Parser {
        tokens,
        eof,
        pos: 0,
        depth: 0,
    }
    .unit()
}

/// Where a statement appears, which decides whether `machine` is legal there.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Scope {
    /// Top level of a file.
    File,
    /// Inside a `machine`, `template` or `for` body.
    Block,
}

struct Parser {
    tokens: Vec<Token>,
    /// Returned by [`Parser::cur`] past the end of the stream. The lexer always
    /// emits an `Eof` token and `advance` stops on it, so this is unreachable —
    /// it exists so that the parser contains no indexing operation at all, and
    /// therefore no way for a bug to become a panic on user input.
    eof: Token,
    pos: usize,
    depth: u32,
}

impl Parser {
    /// The current token.
    fn cur(&self) -> &Token {
        self.tokens.get(self.pos).unwrap_or(&self.eof)
    }

    fn span(&self) -> Span {
        self.cur().span
    }

    fn advance(&mut self) -> Token {
        let tok = self.cur().clone();
        if !matches!(tok.kind, TokenKind::Eof) {
            self.pos += 1;
        }
        tok
    }

    /// Whether the current token is of this kind, ignoring any payload.
    fn at(&self, kind: &TokenKind) -> bool {
        core::mem::discriminant(&self.cur().kind) == core::mem::discriminant(kind)
    }

    fn eat(&mut self, kind: &TokenKind) -> bool {
        if self.at(kind) {
            self.advance();
            true
        } else {
            false
        }
    }

    /// The current token's text, if it is an identifier.
    fn ident_text(&self) -> Option<&str> {
        match &self.cur().kind {
            TokenKind::Ident(text) => Some(text),
            _ => None,
        }
    }

    fn at_ident(&self, word: &str) -> bool {
        self.ident_text() == Some(word)
    }

    /// `expected X, found Y` at the current token.
    fn expected(&self, what: &str) -> Diagnostic {
        Diagnostic::new(
            self.span(),
            format!("expected {what}, found {}", self.cur().kind.describe()),
        )
    }

    fn expect(&mut self, kind: &TokenKind, what: &str) -> Result<Token, Diagnostic> {
        if self.at(kind) {
            Ok(self.advance())
        } else {
            Err(self.expected(what))
        }
    }

    /// Consume a contextual keyword such as `size` or `in`.
    fn expect_keyword(&mut self, word: &str) -> Result<Token, Diagnostic> {
        if self.at_ident(word) {
            Ok(self.advance())
        } else {
            Err(self.expected(&format!("`{word}`")))
        }
    }

    fn expect_string(&mut self, what: &str) -> Result<Spanned<String>, Diagnostic> {
        match &self.cur().kind {
            TokenKind::Str(text) => {
                let value = text.clone();
                let tok = self.advance();
                Ok(Spanned::new(value, tok.span))
            }
            _ => Err(self.expected(what)),
        }
    }

    /// Enter one level of nesting.
    fn enter(&mut self, span: Span) -> Result<(), Diagnostic> {
        self.depth += 1;
        if self.depth > MAX_DEPTH {
            return Err(Diagnostic::new(
                span,
                format!("nested more than {MAX_DEPTH} levels deep"),
            ));
        }
        Ok(())
    }

    fn leave(&mut self) {
        self.depth = self.depth.saturating_sub(1);
    }

    // ---- statements ------------------------------------------------------

    fn unit(mut self) -> Result<SourceUnit, Diagnostic> {
        let start = self.span().start;
        let mut stmts = Vec::new();
        while !self.at(&TokenKind::Eof) {
            stmts.push(self.stmt(Scope::File)?);
        }
        let end = self.span().end;
        Ok(SourceUnit {
            stmts,
            span: Span::new(start, end),
        })
    }

    /// A `{ … }` body of statements. `open` is the brace's span, so that an
    /// unclosed block can point at it.
    fn block(&mut self, open: Span) -> Result<Vec<Stmt>, Diagnostic> {
        self.enter(open)?;
        let mut stmts = Vec::new();
        loop {
            if self.eat(&TokenKind::RBrace) {
                self.leave();
                return Ok(stmts);
            }
            if self.at(&TokenKind::Eof) {
                return Err(self
                    .expected("`}`")
                    .with_note(open, "this `{` is never closed"));
            }
            stmts.push(self.stmt(Scope::Block)?);
        }
    }

    fn stmt(&mut self, scope: Scope) -> Result<Stmt, Diagnostic> {
        let Some(word) = self.ident_text() else {
            return Err(self.expected("a statement"));
        };
        let word = word.to_string();
        let start = self.span();

        match word.as_str() {
            "machine" if scope == Scope::File => self.machine_stmt(start),
            "machine" => Err(Diagnostic::new(
                start,
                "a `machine` block cannot be nested inside another block",
            )),
            "param" => self.param_stmt(start),
            "osc" => self.osc_stmt(start),
            "space" => self.space_stmt(start),
            "object" => self.object_stmt(start),
            "map" => self.map_stmt(start),
            "wire" => self.wire_stmt(start),
            "include" => self.include_stmt(start),
            "template" => self.template_stmt(start),
            "instance" => self.instance_stmt(start),
            "for" => self.for_stmt(start),
            other => Err(Diagnostic::new(
                start,
                format!(
                    "unknown statement `{other}`; expected one of {}",
                    keyword_list()
                ),
            )),
        }
    }

    fn machine_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let name = self.expect_string("a quoted machine name")?;
        let open = self.expect(&TokenKind::LBrace, "`{`")?.span;
        let body = self.block(open)?;
        Ok(Stmt::Machine(MachineDecl {
            name,
            body,
            span: start.join(self.prev_span()),
        }))
    }

    fn param_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let name = self.name()?;
        let default = if self.eat(&TokenKind::Eq) {
            Some(self.expr()?)
        } else {
            None
        };
        Ok(Stmt::Param(ParamDecl {
            name,
            default,
            span: start.join(self.prev_span()),
        }))
    }

    fn osc_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let name = self.name()?;
        self.expect(&TokenKind::Eq, "`=`")?;
        let freq = self.expr()?;
        let unit_tok = self.cur().clone();
        let unit = match self.ident_text().and_then(FreqUnit::from_spelling) {
            Some(unit) => {
                self.advance();
                Spanned::new(unit, unit_tok.span)
            }
            None => {
                return Err(self.expected("a frequency unit (`Hz`, `kHz`, `MHz` or `GHz`)"));
            }
        };
        Ok(Stmt::Osc(OscDecl {
            name,
            freq,
            unit,
            span: start.join(self.prev_span()),
        }))
    }

    fn space_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let name = self.name()?;
        let open = self.expect(&TokenKind::LBrace, "`{`")?.span;
        let props = self.props(open)?;
        Ok(Stmt::Space(SpaceDecl {
            name,
            props,
            span: start.join(self.prev_span()),
        }))
    }

    fn object_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let name = self.name()?;
        let class = self.expect_string("a quoted device class")?;
        // The block is optional: a device with no properties is legitimate.
        let props = if self.at(&TokenKind::LBrace) {
            let open = self.advance().span;
            self.props(open)?
        } else {
            Vec::new()
        };
        Ok(Stmt::Object(ObjectDecl {
            name,
            class,
            props,
            span: start.join(self.prev_span()),
        }))
    }

    fn map_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let space = self.name()?;
        let base = self.expr()?;
        self.expect_keyword("size")?;
        let size = self.expr()?;
        self.expect(&TokenKind::Eq, "`=`")?;
        let target = self.expr()?;
        // An optional trailing block carries per-mapping attributes such as
        // priority and endianness (§4.1). Nothing consumes them yet.
        let props = if self.at(&TokenKind::LBrace) {
            let open = self.advance().span;
            self.props(open)?
        } else {
            Vec::new()
        };
        Ok(Stmt::Map(MapStmt {
            space,
            base,
            size,
            target,
            props,
            span: start.join(self.prev_span()),
        }))
    }

    fn wire_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let from = self.path()?;
        self.expect(&TokenKind::Arrow, "`->`")?;
        let to = self.path()?;
        Ok(Stmt::Wire(WireStmt {
            from,
            to,
            span: start.join(self.prev_span()),
        }))
    }

    fn include_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let path = self.expect_string("a quoted path")?;
        Ok(Stmt::Include(IncludeStmt {
            path,
            span: start.join(self.prev_span()),
        }))
    }

    fn template_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let name = self.name()?;
        let mut params = Vec::new();
        if self.eat(&TokenKind::LParen) {
            loop {
                if self.eat(&TokenKind::RParen) {
                    break;
                }
                let pname = self.name()?;
                let default = if self.eat(&TokenKind::Eq) {
                    Some(self.expr()?)
                } else {
                    None
                };
                let span = pname.span.join(self.prev_span());
                params.push(TemplateParam {
                    name: pname,
                    default,
                    span,
                });
                if !self.eat(&TokenKind::Comma) && !self.at(&TokenKind::RParen) {
                    return Err(self.expected("`,` or `)`"));
                }
            }
        }
        let open = self.expect(&TokenKind::LBrace, "`{`")?.span;
        let body = self.block(open)?;
        Ok(Stmt::Template(TemplateDecl {
            name,
            params,
            body,
            span: start.join(self.prev_span()),
        }))
    }

    fn instance_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let name = self.name()?;
        self.expect(&TokenKind::Eq, "`=`")?;
        let template = self.name()?;
        let mut args = Vec::new();
        if self.eat(&TokenKind::LParen) {
            loop {
                if self.eat(&TokenKind::RParen) {
                    break;
                }
                let arg_start = self.span();
                // `id = 0` binds a parameter; a bare expression is positional.
                let arg_name = if self.ident_text().is_some()
                    && matches!(self.peek_kind(1), Some(TokenKind::Eq))
                {
                    let n = self.name()?;
                    self.advance(); // `=`
                    Some(n)
                } else {
                    None
                };
                let value = self.expr()?;
                args.push(Arg {
                    name: arg_name,
                    value,
                    span: arg_start.join(self.prev_span()),
                });
                if !self.eat(&TokenKind::Comma) && !self.at(&TokenKind::RParen) {
                    return Err(self.expected("`,` or `)`"));
                }
            }
        }
        Ok(Stmt::Instance(InstanceStmt {
            name,
            template,
            args,
            span: start.join(self.prev_span()),
        }))
    }

    fn for_stmt(&mut self, start: Span) -> Result<Stmt, Diagnostic> {
        self.advance();
        let var = self.name()?;
        self.expect_keyword("in")?;
        let from = self.expr()?;
        let inclusive = if self.eat(&TokenKind::DotDotEq) {
            true
        } else {
            self.expect(&TokenKind::DotDot, "`..` or `..=`")?;
            false
        };
        let to = self.expr()?;
        let open = self.expect(&TokenKind::LBrace, "`{`")?.span;
        let body = self.block(open)?;
        Ok(Stmt::For(ForStmt {
            var,
            start: from,
            end: to,
            inclusive,
            body,
            span: start.join(self.prev_span()),
        }))
    }

    /// A `name = value` block, the opening brace already consumed.
    fn props(&mut self, open: Span) -> Result<Vec<Property>, Diagnostic> {
        self.enter(open)?;
        let mut props = Vec::new();
        loop {
            // Commas are separators, and optional; extra ones are harmless.
            while self.eat(&TokenKind::Comma) {}
            if self.eat(&TokenKind::RBrace) {
                self.leave();
                return Ok(props);
            }
            if self.at(&TokenKind::Eof) {
                return Err(self
                    .expected("`}`")
                    .with_note(open, "this `{` is never closed"));
            }
            if self.ident_text().is_none() && !self.at(&TokenKind::Dollar) {
                return Err(self.expected("a property name or `}`"));
            }
            let name = self.name()?;
            self.expect(&TokenKind::Eq, "`=`")?;
            let value = self.expr()?;
            let span = name.span.join(value.span());
            props.push(Property { name, value, span });
        }
    }

    // ---- names, paths, expressions ---------------------------------------

    /// The span of the token just consumed, for closing a statement's span.
    fn prev_span(&self) -> Span {
        match self.pos.checked_sub(1).and_then(|i| self.tokens.get(i)) {
            Some(tok) => tok.span,
            None => self.span(),
        }
    }

    fn peek_kind(&self, ahead: usize) -> Option<&TokenKind> {
        self.tokens.get(self.pos + ahead).map(|t| &t.kind)
    }

    /// A name, possibly with `$` substitutions. Parts must be adjacent in the
    /// source: `cpu$i` is one name, `cpu $i` is two things.
    fn name(&mut self) -> Result<Name, Diagnostic> {
        let start = self.span();
        let mut parts = Vec::new();
        let mut end = start.start;
        loop {
            let tok_span = self.span();
            if !parts.is_empty() && tok_span.start != end {
                break;
            }
            match self.cur().kind.clone() {
                TokenKind::Ident(text) => {
                    self.advance();
                    parts.push(NamePart::Literal(text));
                    end = tok_span.end;
                }
                TokenKind::Dollar => {
                    let dollar = self.advance().span;
                    if self.span().start != dollar.end {
                        return Err(self.expected("a name or `{` directly after `$`"));
                    }
                    match self.cur().kind.clone() {
                        TokenKind::Ident(text) => {
                            let span = self.advance().span;
                            parts.push(NamePart::Substitution(Expr::Path(Path {
                                segments: alloc::vec![Name {
                                    parts: alloc::vec![NamePart::Literal(text)],
                                    span,
                                }],
                                span,
                            })));
                            end = span.end;
                        }
                        TokenKind::LBrace => {
                            self.advance();
                            let expr = self.expr()?;
                            let close = self.expect(&TokenKind::RBrace, "`}`")?.span;
                            parts.push(NamePart::Substitution(expr));
                            end = close.end;
                        }
                        _ => return Err(self.expected("a name or `{` directly after `$`")),
                    }
                }
                _ => break,
            }
        }
        if parts.is_empty() {
            return Err(self.expected("a name"));
        }
        Ok(Name {
            parts,
            span: Span::new(start.start, end),
        })
    }

    fn path(&mut self) -> Result<Path, Diagnostic> {
        let first = self.name()?;
        let start = first.span;
        let mut segments = alloc::vec![first];
        let mut end = start.end;
        while self.at(&TokenKind::Dot) {
            self.advance();
            let seg = self.name()?;
            end = seg.span.end;
            segments.push(seg);
        }
        Ok(Path {
            segments,
            span: Span::new(start.start, end),
        })
    }

    fn expr(&mut self) -> Result<Expr, Diagnostic> {
        self.additive()
    }

    fn additive(&mut self) -> Result<Expr, Diagnostic> {
        let mut lhs = self.multiplicative()?;
        loop {
            let op = if self.at(&TokenKind::Plus) {
                BinOp::Add
            } else if self.at(&TokenKind::Minus) {
                BinOp::Sub
            } else {
                return Ok(lhs);
            };
            self.advance();
            let rhs = self.multiplicative()?;
            let span = lhs.span().join(rhs.span());
            lhs = Expr::Binary {
                op,
                lhs: Box::new(lhs),
                rhs: Box::new(rhs),
                span,
            };
        }
    }

    fn multiplicative(&mut self) -> Result<Expr, Diagnostic> {
        let mut lhs = self.unary()?;
        loop {
            let op = if self.at(&TokenKind::Star) {
                BinOp::Mul
            } else if self.at(&TokenKind::Slash) {
                BinOp::Div
            } else if self.at(&TokenKind::Percent) {
                BinOp::Rem
            } else {
                return Ok(lhs);
            };
            self.advance();
            let rhs = self.unary()?;
            let span = lhs.span().join(rhs.span());
            lhs = Expr::Binary {
                op,
                lhs: Box::new(lhs),
                rhs: Box::new(rhs),
                span,
            };
        }
    }

    /// The one place every expression nesting level passes through, and so the
    /// place the depth guard lives.
    fn unary(&mut self) -> Result<Expr, Diagnostic> {
        self.enter(self.span())?;
        let out = self.unary_inner();
        self.leave();
        out
    }

    fn unary_inner(&mut self) -> Result<Expr, Diagnostic> {
        if self.at(&TokenKind::Minus) {
            let op_span = self.advance().span;
            let operand = self.unary()?;
            let span = op_span.join(operand.span());
            return Ok(Expr::Unary {
                op: UnOp::Neg,
                operand: Box::new(operand),
                span,
            });
        }
        self.primary()
    }

    fn primary(&mut self) -> Result<Expr, Diagnostic> {
        let span = self.span();
        match self.cur().kind.clone() {
            TokenKind::Num(lit) => {
                self.advance();
                Ok(Expr::Num(Spanned::new(lit, span)))
            }
            TokenKind::Str(text) => {
                self.advance();
                Ok(Expr::Str(Spanned::new(text, span)))
            }
            TokenKind::LParen => {
                self.advance();
                let inner = self.expr()?;
                self.expect(&TokenKind::RParen, "`)`")?;
                Ok(inner)
            }
            TokenKind::LBracket => {
                self.advance();
                let mut items = Vec::new();
                loop {
                    while self.eat(&TokenKind::Comma) {}
                    if self.at(&TokenKind::RBracket) {
                        break;
                    }
                    if self.at(&TokenKind::Eof) {
                        return Err(self
                            .expected("`]`")
                            .with_note(span, "this `[` is never closed"));
                    }
                    items.push(self.expr()?);
                }
                let close = self.expect(&TokenKind::RBracket, "`]`")?.span;
                Ok(Expr::List {
                    items,
                    span: span.join(close),
                })
            }
            TokenKind::LBrace => {
                self.advance();
                let entries = self.props(span)?;
                Ok(Expr::Map {
                    entries,
                    span: span.join(self.prev_span()),
                })
            }
            TokenKind::Ident(word) if word == "true" || word == "false" => {
                // `true`/`false` are values, not names — but only when they
                // stand alone, so an object may still be called `true.thing`.
                if matches!(self.peek_kind(1), Some(TokenKind::Dot)) {
                    return self.path_or_call();
                }
                self.advance();
                Ok(Expr::Bool(Spanned::new(word == "true", span)))
            }
            TokenKind::Ident(_) | TokenKind::Dollar => self.path_or_call(),
            _ => Err(self.expected("a value")),
        }
    }

    fn path_or_call(&mut self) -> Result<Expr, Diagnostic> {
        let path = self.path()?;
        let start = path.span;
        // A call's parenthesis must touch its callee. That is what keeps
        // `clock = master` followed on the next line by `(something)` from
        // silently becoming a call, without making newlines significant.
        if self.at(&TokenKind::LParen) && self.span().start == path.span.end {
            let open = self.advance().span;
            let mut args = Vec::new();
            loop {
                while self.eat(&TokenKind::Comma) {}
                if self.at(&TokenKind::RParen) {
                    break;
                }
                if self.at(&TokenKind::Eof) {
                    return Err(self
                        .expected("`)`")
                        .with_note(open, "this `(` is never closed"));
                }
                args.push(self.expr()?);
            }
            let close = self.expect(&TokenKind::RParen, "`)`")?.span;
            return Ok(Expr::Call {
                callee: path,
                args,
                span: start.join(close),
            });
        }
        Ok(Expr::Path(path))
    }
}

/// The statement keywords, formatted for a diagnostic.
fn keyword_list() -> String {
    let mut out = String::new();
    for (i, word) in STATEMENT_KEYWORDS.iter().enumerate() {
        if i > 0 {
            out.push_str(", ");
        }
        out.push('`');
        out.push_str(word);
        out.push('`');
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::machine::ast::NamePart;

    fn dump(text: &str) -> String {
        let src = SourceFile::new("t.machine", text);
        parse(&src).expect("should parse").dump()
    }

    fn err(text: &str) -> String {
        let src = SourceFile::new("t.machine", text);
        parse(&src).expect_err("should fail").message
    }

    #[test]
    fn statements_round_trip_through_the_dumper() {
        assert_eq!(
            dump("param region = \"ntsc\"\n"),
            "param region = \"ntsc\"\n"
        );
        assert_eq!(dump("param ram\n"), "param ram\n");
        assert_eq!(
            dump("osc master = 236250000/11 Hz\n"),
            "osc master = (236250000 / 11) Hz\n"
        );
        assert_eq!(
            dump("space cpubus { width = 16, unassigned = open-bus }\n"),
            "space cpubus { width = 16, unassigned = open-bus }\n"
        );
        assert_eq!(
            dump("object ram \"wram\" { size = 2K }\n"),
            "object ram \"wram\" { size = 2048 }\n"
        );
        assert_eq!(dump("object ram \"wram\"\n"), "object ram \"wram\" {}\n");
        assert_eq!(
            dump("map cpubus 0x0000 size 0x2000 = mirror(wram)\n"),
            "map cpubus 0 size 8192 = mirror(wram)\n"
        );
        assert_eq!(
            dump("wire ppu.nmi -> cpu.nmi\n"),
            "wire ppu.nmi -> cpu.nmi\n"
        );
        assert_eq!(
            dump("include \"pci-common.machine\"\n"),
            "include \"pci-common.machine\"\n"
        );
    }

    #[test]
    fn precedence_is_the_usual_one() {
        assert_eq!(dump("param x = 1 + 2 * 3\n"), "param x = (1 + (2 * 3))\n");
        assert_eq!(dump("param x = (1 + 2) * 3\n"), "param x = ((1 + 2) * 3)\n");
        assert_eq!(dump("param x = 8 / 2 / 2\n"), "param x = ((8 / 2) / 2)\n");
        assert_eq!(dump("param x = -3 + 1\n"), "param x = ((-3) + 1)\n");
        assert_eq!(dump("param x = 7 % 2\n"), "param x = (7 % 2)\n");
    }

    #[test]
    fn values_cover_the_property_types() {
        assert_eq!(dump("param x = true\n"), "param x = true\n");
        assert_eq!(dump("param x = false\n"), "param x = false\n");
        assert_eq!(dump("param x = [1, 2, 3]\n"), "param x = [1, 2, 3]\n");
        assert_eq!(dump("param x = []\n"), "param x = []\n");
        assert_eq!(
            dump("param x = { a = 1, b = \"two\" }\n"),
            "param x = { a = 1, b = \"two\" }\n"
        );
        assert_eq!(dump("param x = 10ms\n"), "param x = 10000000\n");
        assert_eq!(dump("param x = cpu.irq\n"), "param x = cpu.irq\n");
    }

    #[test]
    fn commas_between_properties_are_optional() {
        let with = dump("object cpu \"mos6502\" { clock = master / 12, space = cpubus }\n");
        let without =
            dump("object cpu \"mos6502\" {\n  clock = master / 12\n  space = cpubus\n}\n");
        assert_eq!(with, without);
        // A trailing comma is fine too.
        assert_eq!(dump("space s { width = 8, }\n"), "space s { width = 8 }\n");
    }

    #[test]
    fn a_bare_expression_never_swallows_the_next_statement() {
        // `master` must not absorb the `(` on the following line as a call.
        let text = "object cpu \"c\" { clock = master }\nparam x = 1\n";
        assert_eq!(
            dump(text),
            "object cpu \"c\" { clock = master }\nparam x = 1\n"
        );
    }

    #[test]
    fn templates_loops_and_instances_parse() {
        let text = "\
template cpu_complex(id, clock = master / 12) {
  object cpu$id \"mos6502\" { clock = clock }
  wire cpu$id.irq -> pic.in$id
}
for i in 0..4 {
  instance core$i = cpu_complex(id = i, clock = master / 12)
}
for j in 0..=3 { object bank${j + 1} \"ram\" }
";
        assert_eq!(
            dump(text),
            "\
template cpu_complex(id, clock = (master / 12)) {
  object cpu$id \"mos6502\" { clock = clock }
  wire cpu$id.irq -> pic.in$id
}
for i in 0..4 {
  instance core$i = cpu_complex(id = i, clock = (master / 12))
}
for j in 0..=3 {
  object bank${(j + 1)} \"ram\" {}
}
"
        );
    }

    #[test]
    fn a_template_may_take_no_parameters() {
        assert_eq!(
            dump("template t { object a \"b\" }\ninstance x = t\n"),
            "template t() {\n  object a \"b\" {}\n}\ninstance x = t()\n"
        );
    }

    #[test]
    fn positional_instance_arguments_are_allowed() {
        assert_eq!(dump("instance x = t(1, 2)\n"), "instance x = t(1, 2)\n");
    }

    #[test]
    fn substituted_names_keep_their_parts() {
        let src = SourceFile::new("t", "for i in 0..2 { object cpu$i \"c\" }\n");
        let unit = parse(&src).expect("should parse");
        let Stmt::For(f) = &unit.stmts[0] else {
            panic!("not a for");
        };
        let Stmt::Object(obj) = &f.body[0] else {
            panic!("not an object");
        };
        assert_eq!(obj.name.parts.len(), 2);
        assert!(matches!(&obj.name.parts[0], NamePart::Literal(t) if t == "cpu"));
        assert!(matches!(&obj.name.parts[1], NamePart::Substitution(_)));
        assert_eq!(obj.name.as_literal(), None);
        // The span covers `cpu$i`, not just `cpu`.
        assert_eq!(
            &src.text()[obj.name.span.start as usize..obj.name.span.end as usize],
            "cpu$i"
        );
    }

    #[test]
    fn spans_point_at_what_was_written() {
        let text = "machine \"nes\" {\n  object cpu \"mos6502\" { clock = master / 12 }\n}\n";
        let src = SourceFile::new("t", text);
        let unit = parse(&src).expect("should parse");
        let Stmt::Machine(m) = &unit.stmts[0] else {
            panic!("not a machine");
        };
        assert_eq!(src.location(m.span.start).line, 1);
        assert_eq!(src.location(m.span.end).line, 3);
        let Stmt::Object(obj) = &m.body[0] else {
            panic!("not an object");
        };
        assert_eq!(
            &text[obj.class.span.start as usize..obj.class.span.end as usize],
            "\"mos6502\""
        );
        let value = &obj.props[0].value;
        assert_eq!(
            &text[value.span().start as usize..value.span().end as usize],
            "master / 12"
        );
    }

    #[test]
    fn parse_errors_are_specific() {
        assert_eq!(
            err("machine \"nes\" {\n"),
            "expected `}`, found end of file"
        );
        assert!(
            err("frobnicate x\n")
                .starts_with("unknown statement `frobnicate`; expected one of `machine`")
        );
        assert_eq!(
            err("machine nes {}\n"),
            "expected a quoted machine name, found `nes`"
        );
        assert_eq!(
            err("osc master = 12\n"),
            "expected a frequency unit (`Hz`, `kHz`, `MHz` or `GHz`), found end of file"
        );
        assert_eq!(
            err("osc master = 12 Hertz\n"),
            "expected a frequency unit (`Hz`, `kHz`, `MHz` or `GHz`), found `Hertz`"
        );
        assert_eq!(
            err("map cpubus 0 0x20 = ram\n"),
            "expected `size`, found a number"
        );
        assert_eq!(err("wire a b\n"), "expected `->`, found `b`");
        assert_eq!(
            err("space s { 12 = 4 }\n"),
            "expected a property name or `}`, found a number"
        );
        assert_eq!(err("space s { width = }\n"), "expected a value, found `}`");
        assert_eq!(
            err("param x = 1 +\n"),
            "expected a value, found end of file"
        );
        assert_eq!(
            err("machine \"a\" { machine \"b\" {} }\n"),
            "a `machine` block cannot be nested inside another block"
        );
        assert_eq!(
            err("for i in 0 4 {}\n"),
            "expected `..` or `..=`, found a number"
        );
        assert_eq!(err("}\n"), "expected a statement, found `}`");
        assert_eq!(
            err("param x = $ i\n"),
            "expected a name or `{` directly after `$`, found `i`"
        );
    }

    #[test]
    fn nesting_is_capped_rather_than_overflowing() {
        let mut text = String::from("param x = ");
        for _ in 0..500 {
            text.push('(');
        }
        assert!(err(&text).starts_with("nested more than 64 levels deep"));

        let mut blocks = String::new();
        for _ in 0..500 {
            blocks.push_str("template t { ");
        }
        assert!(err(&blocks).starts_with("nested more than 64 levels deep"));

        // Unary chains recurse too.
        let mut minus = String::from("param x = ");
        for _ in 0..500 {
            minus.push('-');
        }
        minus.push('1');
        assert!(err(&minus).starts_with("nested more than 64 levels deep"));
    }

    #[test]
    fn depth_is_released_so_wide_files_are_fine() {
        // 200 sibling blocks, each shallow: the guard must not accumulate.
        let mut text = String::new();
        for _ in 0..200 {
            text.push_str("space s { a = (1 + (2 * 3)) }\n");
        }
        let src = SourceFile::new("t", &text);
        assert!(parse(&src).is_ok());
    }
}
