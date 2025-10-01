import tkinter as tk
from tkinter import ttk
from abc import abstractmethod
from dataclasses import dataclass, field, fields, is_dataclass
from typing import List, Optional, Dict, Union
import numpy as np


class GroundParamFrame(tk.Frame):
    """ Frame to build pages for setting paramters for classes. Inherit from 
        this frame, and draw it. The rest should be handled by this foundation.

        @param: The parameters that should be modifiable by the frame. Dictionary
            of str: float|np.ndarray
    """
    base_width: int = 30
    base_padding_x: int = 1
    base_padding_y: int = 1
    base_entry_width: int = 10

    def __init__(self,
                 parent,
                 param:Dict):

        super().__init__(parent)
        self.vars = {}
        self.param = param

        for key in self.param.keys():
            self.set_var(key)

        self.canvas = tk.Frame(self)
        self.canvas.pack(fill="both", pady=2)

        # Apply and reset buttons are standard
        frame = tk.Frame(self)
        frame.pack(fill="both", pady=2)
        self.reset_btn = tk.Button(frame, text="Reset", command=self.reset)
        self.apply_btn = tk.Button(frame, text="Apply", command=self.apply)

        self.reset_btn.pack(side="left", padx=10)
        self.apply_btn.pack(side="left", padx=10)

    @staticmethod
    def _create_vars_from_arr(arr):
        """ Recursively create an array of Stringvars that match 
            the dimension of the array, and with the same value
            arr.
        """
        if isinstance(arr, np.ndarray):
            return [GroundParamFrame._create_vars_from_arr(x) for x in arr]
        else:
            return tk.StringVar(value=str(arr))

    def set_var(self, key: str):
        val = self.param[key]
        assert isinstance(val, float) or isinstance(val, np.ndarray), \
            f"Only floats and numpy arrays are supported, not {type(val)}"

        if isinstance(val, float):
            self.vars[key] = tk.StringVar(value=str(val))
            return

        if isinstance(val, np.ndarray):
            self.vars[key] = self._create_vars_from_arr(val)
            return

    @staticmethod
    def _get_arr_from_vars(vars):
        """ Recursively build a list of floats corresponding to 
            the values in the list of stringvars.
        """
        if isinstance(vars, tk.StringVar):
            try:
                return float(vars.get())
            except ValueError:
                raise ValueError(f"Invalid input: '{vars.get()}' is not a valid number.")

        else:
            return [GroundParamFrame._get_arr_from_vars(x) for x in vars]


    @abstractmethod
    def draw_frame(self):
        """
            The layout of the frame is determined in this function
        """
        pass


    @abstractmethod
    def reset(self):
        """ Reset the parameters value to default values,
            display them in the entries, and apply the change.
        """
        # Reset default values in subclass
        self.update_values()
        self.apply()


    def apply(self):
        """ Store the values from the stringvars in the param dictionary.
        """
        for key in self.param.keys():
            val = self._get_arr_from_vars(self.param[key])
            if isinstance(val, float):
                self.param[key] = val
            else: 
                self.param[key] = np.array(val)


    def update_values(self, param=None):
        """ Update the values of the entries associated with each 
            member of the parameter dataclass.
        """
        if param is None and self.param is None:
            return
        elif param is None:
            param = self.param

        for key in param.keys():
            self.set_var(key)


    def add_divider(self, frame=None) -> None:
        """
            Adds a simple line as a divider to the frame.
        """
        frame = frame if frame is not None else self
        # div = tk.Frame(self, bg="black")
        div = tk.Frame(frame, bg="black")
        div.pack(side="top", fill="x", pady=2)


    def add_field(self,
                  text: str,
                  text_var: tk.StringVar,
                  frame: Optional[tk.Frame]=None) -> None:
        """
            Adds a simple Text and Entry row to the frame.
            Entry Text: [    ]
        """
        return self.add_list_field(text, [text_var], frame=frame)


    def add_list_field(self,
                       text: str,
                       text_var: List[tk.StringVar],
                       frame: Optional[tk.Frame]=None) -> None:
        frame = self.canvas if frame is None else frame
        row = tk.Frame(frame)
        row.pack(side="top",
                 fill="x",
                 pady=self.base_padding_y,
                 padx=self.base_padding_x)

        label = tk.Label(row, text=text, width=self.base_width, anchor="nw")
        label.pack(side="left",
                   fill="x",
                   padx=self.base_padding_x,
                   pady=self.base_padding_y)

        for var in text_var:
            tk.Entry(row,
                     width=self.base_entry_width,
                     textvariable=var).pack(side=tk.LEFT, 
                                            fill="x",
                                            pady=self.base_padding_y,
                                            padx=self.base_padding_x)


    def add_array_field(self,
                        text: str,
                        text_vars: List[List[tk.StringVar]],
                        frame: Optional[tk.Frame]=None):

        frame = self.canvas if frame is None else frame
        for i, r in enumerate(text_vars):
            row = tk.Frame(frame)
            row.pack(side="top",
                     fill="x",
                     pady=self.base_padding_y,
                     padx=self.base_padding_x)

            txt = ""
            if i == (len(r) // 2):
                txt = text
            self.add_list_field(txt, r, frame)




