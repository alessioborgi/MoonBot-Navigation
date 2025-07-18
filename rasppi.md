# Connect to the Raspberry pi

1. Connect to the RANDOMWALK3_2.4GHz wifi
2. Then open a terminal and type:
   ```bash
    ssh ubuntu@10.240.20.163
    ```
3. Enter the password: `ubuntu`

# Connect to the Ev3

1. Connect to the RANDOMWALK3_2.4GHz wifi
2. Then open a terminal and type:
   ```bash
    ssh robot@10.240.20.81
    ```
    or 
    ```bash
    ssh robot@ev3dev.local
    ```
    or 
    ```bash
    ssh robot@ev3dev
    ```
3. Enter the password: `maker`


# Test motor A
```bash
ssh robot@10.240.20.81   'brickrun -r -- pybricks-micropython -c "\
from pybricks.ev3devices import Motor; \
from pybricks.parameters import Port; \
Motor(Port.A).run_time(500,2000); \
Motor(Port.A).stop()"'
```