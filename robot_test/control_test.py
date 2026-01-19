# This test file serves the purpose of testing specific features.

def alter_parameters(uart, clazz, parameter):
    uart.write(f"Enter value for {parameter}: \n")
    buf = uart.readline().decode("utf-8", "strict")
    val = float(buf.strip())
    try:
        uart.write(f"Value: '{val}'\n")
        setattr(clazz, parameter, val)
    except AttributeError:
        uart.write(f"Unknown attribute '{parameter}'. Could not assign value '{val}'.")