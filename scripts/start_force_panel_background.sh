#!/usr/bin/env bash
# Keep the loopback panel alive independently of a chat/terminal process.
set -euo pipefail
panel_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if systemctl --user is-active --quiet biped-force-panel.service; then
  echo 'Force panel already running: http://127.0.0.1:8765/'
  exit 0
fi
if [[ -f "$HOME/.config/systemd/user/biped-force-panel.service" ]]; then
  systemctl --user start biped-force-panel.service
  echo 'Force panel started using the installed login service: http://127.0.0.1:8765/'
  exit 0
fi
systemd-run --user --unit=biped-force-panel --collect --property=Restart=on-failure \
  --working-directory="$panel_workspace" "$panel_workspace/scripts/start_force_panel.sh" --port 8765
echo 'Force panel started in background: http://127.0.0.1:8765/'
echo 'Close it with the panel Close button or: systemctl --user stop biped-force-panel.service'
