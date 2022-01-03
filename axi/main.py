import click

import axi

'''
TODO:
axi (repl)
'''

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


@main.command()
@click.argument('width', type=float)
@click.argument('height', type=float)
@click.argument('margin', type=float)
def calibrate_height(width: float, height: float, margin: float):
    device = axi.Device()
    corners = [(margin, margin), (width-margin, height-margin)]
    for corner in corners:
        device.goto(*corner)
        print("Calibrating Pen Up Position")
        while True:
            device.pen_up()
            new_up = input(f"Input new up position (or nothing to continue). Current={device.pen_up_position} ")
            if len(new_up) == 0:
                break
            device.pen_up_position = int(new_up)
            device.configure()
        device.pen_up()
        print("Calibrating Pen Down Position")
        while True:
            device.pen_down()
            new_down = input(f"Input new down position (or nothing to continue). Current={device.pen_down_position} ")
            if len(new_down) == 0:
                break
            device.pen_down_position = int(new_down)
            device.configure()
        device.pen_up()
    device.home()
    device.write_settings()


if __name__ == '__main__':
    main()
