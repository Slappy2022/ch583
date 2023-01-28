FROM debian:bullseye

RUN apt-get update
RUN apt-get install -y \
  curl

RUN curl -fsSL https://deb.nodesource.com/setup_19.x | bash -
RUN apt-get install -y nodejs

RUN curl -o- https://raw.githubusercontent.com/xpack/assets/master/scripts/install-nvm-node-npm-xpm.sh | bash
RUN npm install --global xpm@latest

WORKDIR /opt/gcc-riscv/
RUN xpm init
RUN xpm install @xpack-dev-tools/riscv-none-elf-gcc@latest

RUN apt-get install -y \
  make

WORKDIR /ch583/EVT/EXAM/
COPY EVT/EXAM .
WORKDIR /ch583/EVT/EXAM

CMD /bin/bash
