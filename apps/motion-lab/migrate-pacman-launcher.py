"""One-time local-launcher migration; preserve user files and desktop actions."""
from pathlib import Path
import shutil
import time

home = Path.home()
stamp = time.strftime('%Y%m%d-%H%M%S')
launcher = home / '.local/bin/thor-motion-lab'
desktop = home / '.local/share/applications/thor-motion-lab.desktop'
expected = 'exec /bin/sh "$HOME/.local/share/thor-motion-lab/run-motion-lab" "$@"'
if launcher.exists():
    if expected not in launcher.read_text():
        raise SystemExit('User launcher changed; refusing to overwrite')
    launcher.rename(launcher.with_name(launcher.name + '.before-pacman-' + stamp))
if desktop.exists():
    text = desktop.read_text()
    if text.count('Exec=thor-motion-lab\n') != 1:
        raise SystemExit('Unexpected desktop command; preserve entry without editing')
    shutil.copy2(desktop, desktop.with_name(desktop.name + '.before-pacman-' + stamp))
    desktop.write_text(text.replace('Exec=thor-motion-lab\n', 'Exec=/usr/bin/thor-motion-lab\n', 1))
print('Menu now invokes pacman application; previous launcher preserved.')
