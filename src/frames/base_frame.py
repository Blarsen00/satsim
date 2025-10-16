import tkinter as tk
from tkinter import ttk
from scipy.spatial.transform import Rotation
from abc import abstractmethod
from dataclasses import dataclass, field, fields, is_dataclass
from typing import List, Optional, Dict, Any
import numpy as np


class BaseParamFrame(tk.Frame):
    """
    Foundation frame for building user interfaces designed for setting and
    modifying class parameters using tkinter entry widgets.

    Inherit from this class and implement the abstract methods to create a
    custom parameter page.

    Attributes
    ----------
    base_width : int
        Default width for labels.
    base_padding_x : int
        Default horizontal padding for elements.
    base_padding_y : int
        Default vertical padding for elements.
    base_entry_width : int
        Default width for entry widgets.
    """
    base_width: int = 30
    base_padding_x: int = 1
    base_padding_y: int = 1
    base_entry_width: int = 10

    def __init__(self,
                 parent,
                 param:Dict):
        """
        Initializes the BaseParamFrame.

        Parameters
        ----------
        parent : tk.Widget
            The parent tkinter widget (e.g., tk.Tk or tk.Frame).
        param : Dict[str, float | np.ndarray | Rotation | int]
            A dictionary containing the parameters to be displayed and modified.
            Keys are parameter names (str), and values are the initial parameter data.
        """

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
        self.draw_frame()

    @staticmethod
    def _create_vars_from_arr(arr: Any) -> Any:
        """
        Recursively creates a nested list of tk.StringVar objects that mirrors
        the structure and initial values of a given scalar or numpy array.

        Parameters
        ----------
        arr : Any
            A scalar (float/int) or a nested numpy array.

        Returns
        -------
        Any
            A tk.StringVar if the input is a scalar, or a nested list of tk.StringVar
            if the input is a numpy array.
        """
        if isinstance(arr, np.ndarray):
            return [BaseParamFrame._create_vars_from_arr(x) for x in arr]
        else:
            return tk.StringVar(value=str(arr))

    def set_var(self, key: str) -> None:
        """
        Initializes a tk.StringVar or a nested list of tk.StringVar objects for a
        specific parameter key based on the type of its associated value in `self.param`.

        Handles float, int, numpy.ndarray, and scipy.spatial.transform.Rotation types.
        Rotation objects are converted to quaternions (4-element arrays) before
        creating the StringVars.

        Parameters
        ----------
        key : str
            The key of the parameter in `self.param` to initialize StringVars for.

        Raises
        ------
        AssertionError
            If the parameter value's type is not supported.
        """
        val = self.param[key]
        assert isinstance(val, float) or isinstance(val, np.ndarray) \
                             or isinstance(val, np.floating) \
                             or isinstance(val, Rotation) \
                             or isinstance(val, int), \
            f"Only floats, integers, numpy arrays, and Rotation objects are supported, not {type(val)}"

        if isinstance(val, float) or isinstance(val, np.floating) or isinstance(val, int):
            self.vars[key] = tk.StringVar(value=str(val))
            return

        if isinstance(val, np.ndarray):
            self.vars[key] = self._create_vars_from_arr(val)
            return

        if isinstance(val, Rotation):
            # NOTE: Convert the rotation to a quaternion before making stringvars
            self.vars[key] = self._create_vars_from_arr(val.as_quat())


    @staticmethod
    def _get_arr_from_vars(vars: Any) -> Any:
        """
        Recursively extracts the numerical values from a nested list of tk.StringVar
        objects, converting them to floats.

        Parameters
        ----------
        vars : Any
            A tk.StringVar object or a nested list of them.

        Returns
        -------
        Any
            A float if the input is a tk.StringVar, or a nested list of floats.

        Raises
        ------
        ValueError
            If any tk.StringVar contains a non-numeric string.
        """
        if isinstance(vars, tk.StringVar):
            try:
                return float(vars.get())
            except ValueError:
                raise ValueError(f"Invalid input: '{vars.get()}' is not a valid number.")

        else:
            return [BaseParamFrame._get_arr_from_vars(x) for x in vars]

    @abstractmethod
    def draw_frame(self) -> None:
        """
        Abstract method to be implemented by subclasses. This function defines
        the specific layout and arrangement of labels and entry widgets for the
        parameters on the frame.
        """
        pass

    def redraw(self) -> None:
        """
        Updates the geometry and appearance of the frame.
        """
        self.update()
        # for widget in self.winfo_children():
            # widget.destroy()
        # self.draw_frame()

    @abstractmethod
    def reset(self) -> None:
        """
        Abstract method to be implemented by subclasses.

        Resets the parameters to their default values, updates the entry fields
        to reflect these defaults, and calls `apply()` to commit the changes.
        """
        # Reset default values in subclass
        self.update_values()
        self.apply()

    @abstractmethod
    def get_obj(self) -> Any:
        """
        Abstract method to be implemented by subclasses.

        Returns the object or class instance whose parameters are being configured
        by this frame.
        """
        pass

    def apply(self) -> None:
        """
        Extracts the numerical values from all tk.StringVar objects and updates
        the corresponding entries in the `self.param` dictionary.

        Handles conversion back to float, numpy.ndarray, or Rotation objects.
        Rotation objects are normalized after conversion and their corresponding
        StringVars are updated to display the normalized quaternion (with precision 3).

        Raises
        ------
        TypeError
            If the extracted value type does not match the expected type logic.
        """
        for key in self.param.keys():
            val = self._get_arr_from_vars(self.vars[key])
            if isinstance(val, float) or isinstance(val, int):
                self.param[key] = val
            elif isinstance(self.param[key], Rotation):
                # NOTE: Apply the update after converting to quaternion to display the normalized quaternion
                self.param[key] = Rotation.from_quat(val)
                BaseParamFrame.update_vars(self.vars[key], self.param[key].as_quat(), precision=3)
            elif isinstance(val, list): 
                self.param[key] = np.array(val)
            else:
                raise TypeError()


    @staticmethod
    def update_vars(vars: Any, arr: Any, precision: float=np.inf) -> None:
        """
        Recursively sets the values of tk.StringVar or nested lists of tk.StringVar
        from a source scalar or array, applying optional precision formatting.

        Parameters
        ----------
        vars : Any
            A tk.StringVar object or a nested list of them (the target).
        arr : Any
            A scalar, float, int, or nested numpy array (the source values).
        precision : float, optional
            The number of decimal places to format the string to. If set to
            `np.inf`, standard string conversion is used. Defaults to `np.inf`.

        Raises
        ------
        AssertionError
            If a list of StringVars and the source array are not the same size.
        TypeError
            If an unsupported type is encountered.
        """
        if isinstance(vars, tk.StringVar) and \
            (isinstance(arr, np.floating) or isinstance(arr, float)
                                             or isinstance(arr, int)):
            # NOTE: Apply precision for the normalized quaternion.
            if precision == np.inf:
                vars.set(str(arr))
            else:
                s = f"{arr:.{precision}f}"
                vars.set(s)
            return

        elif isinstance(vars, list) and isinstance(arr, np.ndarray):
            assert len(vars) == len(arr), "vars and values need to be of same size."

            for i in range(len(vars)):
                BaseParamFrame.update_vars(vars[i], arr[i], precision=precision) 
            return

        else:
            print(f"Unsupported type for vars: {type(vars)}")
            print(vars)
            print(arr)
            raise TypeError(f"Unsupported type for vars: {type(vars)}")


    def update_values(self, param: Optional[Dict]=None) -> None:
        """
        Updates the content of all tk.StringVar objects to reflect the current
        values stored in the parameter dictionary (`self.param` or the provided `param`).
        This refreshes the content displayed in the entry widgets.

        Parameters
        ----------
        param : Optional[Dict], optional
            An optional dictionary of parameter values to use for updating.
            If None, uses `self.param`. Defaults to None.
        """
        if param is None and self.param is None:
            return
        elif param is None:
            param = self.param

        for key in param.keys():
            val = param[key]
            if isinstance(val, Rotation):
                BaseParamFrame.update_vars(self.vars[key], val.as_quat())
            else:
                BaseParamFrame.update_vars(self.vars[key], val)

    def add_divider(self, frame: Optional[tk.Frame]=None) -> None:
        """
        Adds a simple black horizontal line as a visual divider to the canvas.

        Parameters
        ----------
        frame : Optional[tk.Frame], optional
            The frame to add the divider to. If None, uses `self.canvas`.
            Defaults to None.
        """
        frame = frame if frame is not None else self.canvas
        div = tk.Frame(frame, bg="black", height=1)
        div.pack(side="top", fill="x", pady=2)


    def add_field(self,
                  text: str,
                  text_var: tk.StringVar,
                  frame: Optional[tk.Frame]=None) -> None:
        """
        Adds a label and a single entry field to the frame.

        Layout: [Label] [Entry]

        Parameters
        ----------
        text : str
            The text label for the field.
        text_var : tk.StringVar
            The StringVar connected to the entry field.
        frame : Optional[tk.Frame], optional
            The frame to add the field to. If None, uses `self.canvas`.
            Defaults to None.
        """
        return self.add_list_field(text, [text_var], frame=frame)


    def add_list_field(self,
                       text: str,
                       text_var: List[tk.StringVar],
                       frame: Optional[tk.Frame]=None) -> None:
        """
        Adds a label followed by a horizontal list of entry fields (e.g., for a vector).

        Layout: [Label] [Entry 1] [Entry 2] ...

        Parameters
        ----------
        text : str
            The text label for the list of fields.
        text_var : List[tk.StringVar]
            A list of StringVars, each connected to an entry field.
        frame : Optional[tk.Frame], optional
            The frame to add the fields to. If None, uses `self.canvas`.
            Defaults to None.
        """
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
        """
        Adds a label and a grid of entry fields (e.g., for a matrix or 2D array).
        The label is intended to be vertically centered next to the array rows.

        Layout:
        [     ] [Entry 1,1] [Entry 1,2] ...
        [Label] [Entry 2,1] [Entry 2,2] ...
        [     ] [Entry 3,1] [Entry 3,2] ...

        Parameters
        ----------
        text : str
            The main label text for the array.
        text_vars : List[List[tk.StringVar]]
            A nested list representing the array structure, where each inner list
            corresponds to a row of entry fields.
        frame : Optional[tk.Frame], optional
            The frame to add the array to. If None, uses `self.canvas`.
            Defaults to None.
        """

        frame = self.canvas if frame is None else frame
        for i, r in enumerate(text_vars):
            row = tk.Frame(frame)
            row.pack(side="top",
                     fill="x",
                     pady=self.base_padding_y,
                     padx=self.base_padding_x)

            txt = ""
            if i == (len(text_vars) // 2):
                txt = text
            self.add_list_field(txt, r, frame)

if __name__ == '__main__':
    root = tk.Tk()
    arr = np.array([
        [[1.0, 1.0, 1.0], [1.0, 5.0, 1.0], [1.0, 1.0, 1.0]],
        [[4.0, 11.0, 1.0], [1.0, 2.0, 1.0], [1.0, 5.0, 1.0]],
        [[1.0, 1.4, 1.0], [5.0, 1.0, 1.0], [1.0, 1.0, 1.0]],
        [[1.0, 1.0, 1.0], [4.0, 1.0, 1.0], [1.3, 4.2, 1.0]],
    ])
    test = BaseParamFrame._create_vars_from_arr(arr)
    test_out = BaseParamFrame._get_arr_from_vars(test)

    print(test)
    print(test_out)
