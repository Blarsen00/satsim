from dataclasses import is_dataclass
from simulation import Simulate, PhysicalState


class BaseReference:
    state: PhysicalState = PhysicalState()

    def __init__(self, state=None) -> None:
        if state is not None:
            self.state = state
        else:
            self.state = PhysicalState()


    def update(self, dt: float):
        """
            Update the reference state for each time step.
        """
        if not BaseReference.check_dataclass(self.state):
            return None

        self.state.w = Simulate.direct_euler(self.state.w, 
                                               self.state.w_dot,
                                               dt)
        self.state.rot = Simulate.calculate_attitude(self.state.rot,
                                                     self.state.w,
                                                     dt)
        return self.state


    @staticmethod
    def check_dataclass(dataclass) -> bool:
        """
            Checks if the dataclass received is of the correct 
            type. It needs to be a dataclass and it needs the correct
            attributes in order to be used.
        """
        if not is_dataclass(dataclass):
            print(f"Error: {dataclass} is not a dataclass")
            return False
        
        wanted_attributes = ["rot", "w", "w_dot"]
        for a in wanted_attributes:
            if not hasattr(dataclass, a):
                print(f"Error: Wrong dataclass, as it has no member {a}")
                return False
        
        return True


