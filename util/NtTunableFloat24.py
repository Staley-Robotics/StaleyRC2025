import typing
from ntcore import NetworkTableInstance, Event, EventFlags

class NTTunableFloat:
    """
    NTTunableFloat is a custom class designed to:
        1. Publish a Float value to NetworkTables (with persistence if requested)
        2. Store the Float value in memory for faster access
        3. Creates a NetworkTables listener to provide ad-hoc updates to the in memory copy
        4. Triggers custom update functions
    """

    table:str = None
    name:str = None
    value:float = 0.0
    updater:typing.Callable[[],None] = lambda: None
    persistent:bool = False

    def __init__( self,
                  name:str,
                  value:float,
                  updater:typing.Callable[[],None] = lambda: None,
                  persistent:bool = False
                ) -> None:
        """
        Initialization
        """
        # Save Global Variables
        if not name.startswith("/"):
            self.rootTbl = "Config"
            self.name = name
        elif len(name) != 1:
            if len(name.split("/")) == 2:
                self.rootTbl = "Config"
                self.name = name.split("/")[1]
            else:
                self.rootTbl = name.split("/")[1]
                self.name = "/".join(name.split("/")[2:])
        else:
            raise Exception( f"{self.__class__.__str__()}: Invalid Name" )
        
        self.updater = updater

        # Get Network Tables
        self.ntTbl = NetworkTableInstance.getDefault().getTable(self.rootTbl)
        
        # Save Value to Network Tables and Memory
        self.value = float( self.ntTbl.getNumber( self.name, value ) )
        if not self.ntTbl.putNumber( self.name, self.value ):
            raise Exception(
                f"{self.__class__.__str__()}: Network Table Value already exists as different type."
            )
        
        # Properly Configure Persistence
        if persistent:
            self.ntTbl.setPersistent( self.name )
        if not persistent:
            self.ntTbl.clearPersistent( self.name )

        # Add Listener
        NetworkTableInstance.getDefault().addListener(
            [f"/{self.rootTbl}/{self.name}"],
            EventFlags.kValueAll,
            self.update
        )

    def get(self) -> float:
        """
        Get the current value from memory.
        """
        return float(self.value)

    def set(self, value:float) -> None:
        """
        Sets the updated value on the NetworkTable and in memory.
        """
        if self.ntTbl.putNumber( self.name, value ):
            self.value = value
        else:
            raise Exception(
                f"{self.__class__.__str__()}: Network Table Value already exists as different type."
            )
    
    def update(self, event:Event) -> None:
        """
        Get the updated value from the NetworkTables Listener.
        Reverts the value if there is a type error.
        """
        try:
            value = float( event.data.value.value() )
            self.value = value
            self.updater()
        except Exception as e:
            if not self.ntTbl.putNumber( self.name, self.value ):
                raise Exception( f"{self.__class__.__str__()}: Network Table Value Update Error." )