fn main() {
    if std::env::var("DOCS_RS").is_ok() {
        // docs.rs用の設定
        return;
    }
    embuild::espidf::sysenv::output();
}
