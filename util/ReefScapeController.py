# Similar to falcon xbox controller
# Creating an overlay for our fancy controller

from commands2.button import CommandJoystick, Trigger


class ReefScapeController:
    """
    This class is a container for our custom ReefScape game controller
    """

    def __init__(self, portA: int, portB: int):
        """
        Constructor for the ReefScapeController class.
        :param portA: The port number for the first half of the console.
        :param portB: The port number for the second half of the console.
        """
        self.ConsoleA = CommandJoystick(portA)
        self.ConsoleB = CommandJoystick(portB)

    def R1(self) -> Trigger: # upper left right
        """
        Returns the C1 designator button on the console.
        """
        return self.ConsoleA.button(10)

    def R2(self) -> Trigger:
        """
        Returns the C2 designator button on the console.
        """
        return self.ConsoleA.button(11)

    def R3(self) -> Trigger:
        """
        Returns the C3 designator button on the console.

        """
        return self.ConsoleA.button(12)

    def R4(self) -> Trigger:
        """
        Returns the C4 designator button on the console.
        """
        return self.ConsoleB.button(12)

    def R5(self) -> Trigger:
        """
        Returns the C5 designator button on the console.
        """
        return self.ConsoleB.button(11)

    def R6(self) -> Trigger:
        """
        Returns the C6 designator button on the console.
        """
        return self.ConsoleB.button(10)

    def R7(self) -> Trigger:
        """
        Returns the C7 designator button on the console.
        """
        return self.ConsoleB.button(9)

    def R8(self) -> Trigger:
        """
        Returns the C8 designator button on the console.
        """
        return self.ConsoleB.button(8)

    def R9(self) -> Trigger:
        """
        Returns the C9 designator button on the console.
        """
        return self.ConsoleB.button(7)

    def R10(self) -> Trigger:
        """
        Returns the C10 designator button on the console.
        """
        return self.ConsoleA.button(7)

    def R11(self) -> Trigger:
        """
        Returns the C11 designator button on the console.
        """
        return self.ConsoleA.button(8)

    def R12(self) -> Trigger:
        """
        Returns the C12 designator button on the console.
        """
        return self.ConsoleA.button(9)


    def L1(self) -> Trigger:
        """
        Returns the L1 designator button on the console.
        """
        return self.ConsoleB.button(3)

    def L2(self) -> Trigger:
        """
        Returns the L2 designator button on the console.
        """
        return self.ConsoleB.button(4)

    def L3(self) -> Trigger:
        """
        Returns the L3 designator button on the console.
        """
        return self.ConsoleB.button(5)

    def L4(self) -> Trigger:
        """
        Returns the L4 designator button on the console.
        """
        return self.ConsoleB.button(6)

    def Inner(self) -> Trigger:
        """
        Returns the Inner Source designator button on the console.
        """
        return self.ConsoleA.button(4)

    def Middle(self) -> Trigger:
        """
        Returns the Middle Source designator button on the console.
        """
        return self.ConsoleA.button(5)

    def Outer(self) -> Trigger:
        """
        Returns the Outer Source designator button on the console.
        """
        return self.ConsoleA.button(3)

    def Reset(self) -> Trigger:
        """
        Returns the Reset designator button on the console.
        """
        return self.ConsoleA.button(6)
