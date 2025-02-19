import typing
from ntcore import NetworkTable, NetworkTableInstance, EventFlags, Event

class BasicTunable:
    '''
    Puts a value on the networkTable, updates that value passively
    '''

    id:str=None
    default_val:float=None
    updater:typing.Callable[[], None]=None
    persistant:bool=False

    def __init__(self, nt_path_name:str, default_val:float, update_func:typing.Callable[[], None], persistant:bool=False):
        ## Get data setup
        self.id = nt_path_name
        self.default_val = default_val
        self.updater = update_func
        self.persistant = persistant

        ## Figure out directory path
        if nt_path_name[0] == '/':
            table = nt_path_name.split('/')[0]
            path = nt_path_name[len(table)+1:]
        else:
            table = 'Tunables'
            path = '/' + nt_path_name

        ## Config NTTable
        self.nt_table = NetworkTableInstance.getDefault().getTable(table)

        # Add value to table
        self.nt_table.putNumber( f'/{table}/{path}' )

        # Persistance
        if persistant:
            self.nt_table.setPersistent(  )

        self.nt_table.addListener( f'/{table}/{path}',
                                  EventFlags.kValueAll,
                                   self.update )

    def update(self, table:NetworkTable, name:str, event:Event) -> None:
        ...

    def periodic(self) -> None:
        ...
    
    def get(self) -> float:
        ...
    
    def set(self, val:float) -> None:
        ...