"""Keyboard reading with pygame (arrow keys, ESC).

The pygame window must have focus for key presses to be seen.
"""
import os

os.environ.setdefault('PYGAME_HIDE_SUPPORT_PROMPT', '1')
import pygame  # noqa: E402

_win = None
_font = None

# Checked in priority order: the first key held wins.
_KEYS = (
    ('LEFT', 'LEFT'),
    ('RIGHT', 'RIGHT'),
    ('UP', 'UP'),
    ('DOWN', 'DOWN'),
)


def init(size=(320, 120), caption='pirobot teleop'):
    global _win, _font
    pygame.init()
    _win = pygame.display.set_mode(size)
    pygame.display.set_caption(caption)
    _font = pygame.font.Font(None, 28)
    show('STOP')


def _pump():
    """Process window events. Returns False if the window was closed."""
    alive = True
    for eve in pygame.event.get():
        if eve.type == pygame.QUIT:
            alive = False
    return alive


def getKey(keyName):
    """True while the named key (pygame K_<name>) is held down."""
    _pump()
    return bool(pygame.key.get_pressed()[getattr(pygame, 'K_{}'.format(keyName))])


def fetch_command():
    """Return 'LEFT' | 'RIGHT' | 'UP' | 'DOWN' | 'STOP', or 'EXIT' on ESC/close."""
    if not _pump():
        return 'EXIT'
    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        return 'EXIT'
    for name, cmd in _KEYS:
        if keys[getattr(pygame, 'K_' + name)]:
            return cmd
    return 'STOP'


def show(cmd):
    """Draw the current command in the window."""
    if _win is None:
        return
    _win.fill((30, 30, 30) if cmd == 'STOP' else (0, 90, 0))
    lines = (cmd, 'arrows: drive   ESC: quit')
    for i, text in enumerate(lines):
        _win.blit(_font.render(text, True, (230, 230, 230)), (10, 15 + 40 * i))
    pygame.display.flip()


def stop():
    pygame.quit()


if __name__ == '__main__':
    init()
    clock = pygame.time.Clock()
    last = None
    while True:
        c = fetch_command()
        if c != last:
            print(c)
            show(c)
            last = c
        if c == 'EXIT':
            break
        clock.tick(30)
    stop()
