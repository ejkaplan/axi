import click

import axi

'''
TODO:
axi (repl)
'''


# def main():
#     args = sys.argv[1:]
#     if len(args) == 0:
#         return
#     command, args = args[0], args[1:]
#     command = command.lower()
#     if command == 'render':
#         d = axi.Drawing.load(args[0])
#         d = d.rotate_and_scale_to_fit(12, 8.5, step=90)
#         path = args[1] if len(args) > 1 else 'out.png'
#         im = d.render()
#         im.write_to_png(path)
#         return
#     device = axi.Device()
#     if command == 'zero':
#         device.zero_position()
#     elif command == 'home':
#         device.home()
#     elif command == 'up':
#         device.pen_up()
#     elif command == 'down':
#         device.pen_down()
#     elif command == 'on':
#         device.enable_motors()
#     elif command == 'off':
#         device.disable_motors()
#     elif command == 'move':
#         dx, dy = map(float, args)
#         device.move(dx, dy)
#     elif command == 'goto':
#         x, y = map(float, args)
#         device.goto(x, y)
#     elif command == 'draw':
#         d = axi.Drawing.load(args[0])
#         axi.draw(d)
#     else:
#         pass

@click.group()
def main():
    ...


@main.command()
def zero():
    axi.Device().zero_position()


@main.command()
def home():
    axi.Device().home()


@main.command()
def up():
    axi.Device().pen_up()


@main.command()
def down():
    axi.Device().pen_down()


@main.command()
def on():
    axi.Device().enable_motors()


@main.command()
def off():
    axi.Device().disable_motors()


@main.command()
@click.argument('dx', type=float)
@click.argument('dy', type=float)
def move(dx: float, dy: float):
    axi.Device().move(dx, dy)


@main.command()
@click.argument('x', type=float)
@click.argument('y', type=float)
def goto(x: float, y: float):
    axi.Device().goto(x, y)


if __name__ == '__main__':
    main()
