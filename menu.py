from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.tools import hub_menu

# Make a menu to choose a letter. You can also use numbers.
from pybricks.tools import hub_menu

select = hub_menu("0", "1", "2", "3","4", "5", "6", "7")
if select == "0":           
    import testmotors
elif select == "1":
    import saida1       
elif select == "2":
    import saida2
elif select == "3":
    import saida3
elif select == "4":
    import saida4
elif select == "5":
    import saida5
elif select == "6":
    import saida6
elif select == "7":
    import saida7