default: verify

verify:
    cargo fmt --all --check
    cargo test --no-default-features
    cargo test --all-features
    cargo clippy --all-targets --no-default-features -- -D warnings
    cargo clippy --all-targets --all-features -- -D warnings
    RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --no-default-features
    RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
    cargo check --lib --target thumbv6m-none-eabi --no-default-features
    cargo check --lib --target thumbv6m-none-eabi --all-features
