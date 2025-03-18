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

    def C1(self) -> Trigger:
        """
        Returns the C1 designator button on the console.
        """
        return self.ConsoleA.button(1)

    def C2(self) -> Trigger:
        """
        Returns the C2 designator button on the console.
        """
        return self.ConsoleA.button(2)

    def C3(self) -> Trigger:
        """
        Returns the C3 designator button on the console.

        """
        return self.ConsoleA.button(3)

    def C4(self) -> Trigger:
        """
        Returns the C4 designator button on the console.
        """
        return self.ConsoleA.button(4)

    def C5(self) -> Trigger:
        """
        Returns the C5 designator button on the console.
        """
        return self.ConsoleA.button(5)

    def C6(self) -> Trigger:
        """
        Returns the C6 designator button on the console.
        """
        return self.ConsoleA.button(6)

    def C7(self) -> Trigger:
        """
        Returns the C7 designator button on the console.
        """
        return self.ConsoleA.button(7)

    def C8(self) -> Trigger:
        """
        Returns the C8 designator button on the console.
        """
        return self.ConsoleA.button(8)

    def C9(self) -> Trigger:
        """
        Returns the C9 designator button on the console.
        """
        return self.ConsoleA.button(9)

    def C10(self) -> Trigger:
        """
        Returns the C10 designator button on the console.
        """
        return self.ConsoleA.button(10)

    def C11(self) -> Trigger:
        """
        Returns the C11 designator button on the console.
        """
        return self.ConsoleA.button(11)

    def C12(self) -> Trigger:
        """
        Returns the C12 designator button on the console.
        """
        return self.ConsoleA.button(12)


    def L1(self) -> Trigger:
        """
        Returns the L1 designator button on the console.
        """
        return self.ConsoleB.button(1)

    def L2(self) -> Trigger:
        """
        Returns the L2 designator button on the console.
        """
        return self.ConsoleB.button(2)

    def L3(self) -> Trigger:
        """
        Returns the L3 designator button on the console.
        """
        return self.ConsoleB.button(3)

    def L4(self) -> Trigger:
        """
        Returns the L4 designator button on the console.
        """
        return self.ConsoleB.button(4)

    def Inner(self) -> Trigger:
        """
        Returns the Inner Source designator button on the console.
        """
        return self.ConsoleB.button(5)

    def Middle(self) -> Trigger:
        """
        Returns the Middle Source designator button on the console.
        """
        return self.ConsoleB.button(6)

    def Outer(self) -> Trigger:
        """
        Returns the Outer Source designator button on the console.
        """
        return self.ConsoleB.button(7)

    def Reset(self) -> Trigger:
        """
        Returns the Reset designator button on the console.
        """
        return self.ConsoleB.button(8)
