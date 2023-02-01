#!/bin/bash
set -eux

main() {
  coproc bluetoothctl
  echo -e "connect 38:3B:26:CC:FF:5A\ngatt.select-attribute /org/bluez/hci0/dev_38_3B_26_CC_FF_5A/service0021/char0022\ngatt.write 0x00\nexit" >&"${COPROC[1]}"
  cat <&${COPROC[0]}
}

main "$@"
