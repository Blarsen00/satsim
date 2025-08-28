import tkinter as tk
from tkinter import ttk
from abc import abstractmethod
from dataclasses import dataclass, field, fields, is_dataclass


class BaseParamFrame(tk.Frame):
    base_width: int = 15
    base_padding_x: int = 5
    base_padding_y: int = 2
    base_entry_width: int = 10

    def __init__(self, parent, param=None):
        super().__init__(parent)
        self.param = param

        self.param_vars = {}
        if self.param is not None:
            for f in fields(self.param):
                var = tk.StringVar(value=getattr(self.param, f.name))
                self.param_vars[f.name] = var

        self.canvas = tk.Frame(self)
        self.canvas.pack(fill="both", pady=2)

        # Apply and reset buttons are standard
        frame = tk.Frame(self)
        frame.pack(fill="both", pady=2)
        self.reset_btn = tk.Button(frame, text="Reset", command=self.reset)
        self.apply_btn = tk.Button(frame, text="Apply", command=self.apply)

        self.reset_btn.pack(side="left", padx=10)
        self.apply_btn.pack(side="left", padx=10)

    @abstractmethod
    def draw_frame(self):
        """
            The layout of the frame is determined in this function
        """
        pass


    @abstractmethod
    def reset(self):
        """
            Reset the parameters value to default values, and 
            display them in the entries.
        """
        pass


    def apply(self):
        """
            Apply the data from the entries to the stored 
            parameters dataclass self.param. The following
            datatypes are supported:
                - float
                - int
                - str
                - bool
        """
        for f in fields(self.param):
            if f.type is float:
                setattr(self.param,
                        f.name, 
                        float(self.param_vars[f.name].get()))
            elif f.type is int:
                setattr(self.param,
                        f.name, 
                        int(self.param_vars[f.name].get()))
            elif f.type is bool:
                setattr(self.param,
                        f.name, 
                        bool(self.param_vars[f.name].get()))
            elif f.type is str:
                setattr(self.param,
                        f.name, 
                        str(self.param_vars[f.name].get()))
        print(self.param)


    def update_values(self, param=None):
        """
            Update the values of the entries associated with each 
            member of the parameter dataclass.
        """
        if param is None and self.param is None:
            return
        elif param is None:
            param = self.param

        for f in fields(param):
            self.param_vars[f.name].set(str(getattr(param, f.name)))


    def add_divider(self) -> None:
        """
            Adds a simple line as a divider to the frame.
        """
        div = tk.Frame(self, bg="black")
        div.pack(side="top", fill="x", pady=2)


    def add_field(self, text: str, text_var: tk.StringVar) -> tk.Entry:
        """
            Adds a simple Text and Entry row to the frame.
            Entry Text: [    ]
        """
        row = tk.Frame(self.canvas)
        row.pack(side="top",
                 fill="x",
                 pady=self.base_padding_y,
                 padx=self.base_padding_x)

        label = tk.Label(row, text=text, width=self.base_width, anchor="w")
        label.pack(side="left",
                   fill="x",
                   padx=self.base_padding_x,
                   pady=self.base_padding_y)

        entry = tk.Entry(row, width=self.base_entry_width, textvariable=text_var)
        entry.pack(side="left", fill="x")
