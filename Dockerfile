FROM rust:1.67
WORKDIR /app
RUN apt-get update -y
RUN apt-get install -y libusb-1.0-0-dev
RUN git clone https://github.com/jcranney/gxccd-wrap
RUN cargo build --release --manifest-path=./gxccd-wrap/Cargo.toml
RUN cargo install --path=./gxccd-wrap
CMD ["gxccd"]