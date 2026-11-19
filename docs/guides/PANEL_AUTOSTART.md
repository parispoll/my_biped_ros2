# Panel startup at login

The experiment panel now uses a persistent **user** systemd service, enabled
at login, instead of a temporary `systemd-run` service. It remains bound to
`127.0.0.1:8765`; this does not expose the website to other computers.

Only the website starts automatically. No experiment or Gazebo simulation
starts until Run is clicked. Saved history stays in the workspace.

Installed service:
`/home/parispoll/.config/systemd/user/biped-force-panel.service`

## Controls

```bash
# Status
systemctl --user status biped-force-panel.service

# Start again after intentionally closing the panel
bash scripts/start_force_panel_background.sh

# Stop for this session (also available as Close panel in the GUI)
systemctl --user stop biped-force-panel.service

# Disable automatic startup and stop the panel
systemctl --user disable --now biped-force-panel.service

# Re-enable automatic startup and start now
systemctl --user enable --now biped-force-panel.service
```

Closing the panel stops it normally; it will start again at the next login
while enabled. Unexpected server failures are restarted after three seconds.
During laptop sleep the server cannot respond; after waking, refresh the tab.
Automatic login startup addresses reboot/logout persistence. It does not
establish why a particular lid-close event ended a session.

## Reinstall or move the project

`python3 scripts/services/install_force_panel_service.py` regenerates the service for
the workspace location. It refuses to interrupt an active panel experiment,
validates the candidate unit before stopping the old service, preserves a
different existing unit in a backup, enables the service, starts it and checks
the state API. It may require permission to write outside the workspace.

The background-start helper now uses an installed persistent service when
present, and retains the temporary-service fallback otherwise. The service
allows up to 45 seconds for the panel to stop its owned experiment processes.

## Verification - 1 November 2026

An active 10 mm GUI experiment was detected and left to finish before
installation. After installation, systemd reported `ActiveState=active`,
`UnitFileState=enabled`, `Transient=no`, with the fragment in the persistent
user configuration directory. The state API returned HTTP 200. The final
unit was validated. A reboot/logout test was not performed during this chat.

An initial WorkingDirectory quoting error was corrected and the installer
was strengthened to verify configuration before stopping a working panel.
Robot geometry, controllers, experiment limits and recorded results were
not changed.
