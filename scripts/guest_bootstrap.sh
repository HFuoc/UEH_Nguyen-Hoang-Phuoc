#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
cat > /tmp/crc-ubuntu.list <<'EOF'
deb https://archive.ubuntu.com/ubuntu focal main universe
deb https://archive.ubuntu.com/ubuntu focal-updates main universe
deb https://security.ubuntu.com/ubuntu focal-security main universe
EOF
APT_SOURCE=(-o Dir::Etc::sourcelist=/tmp/crc-ubuntu.list -o Dir::Etc::sourceparts=- -o Acquire::ForceIPv4=true)
apt-get "${APT_SOURCE[@]}" update
apt-get "${APT_SOURCE[@]}" install -y --no-install-recommends docker.io
systemctl enable --now docker
usermod -aG docker fish
docker version
df -h /
