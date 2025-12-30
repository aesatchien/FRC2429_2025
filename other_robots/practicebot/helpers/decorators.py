import functools
import warnings
import time
import ntcore

import constants

# Pre-allocate the publisher globally for the module
inst = ntcore.NetworkTableInstance.getDefault()
status_prefix = constants.status_prefix
alert_pub = inst.getStringTopic(f"{status_prefix}/alert").publish()


def deprecated(reason):
    """
    A decorator to mark a class or function as deprecated.
    Emits a warning when the class is instantiated or function is called.
    """
    def decorator(cls_or_func):
        if isinstance(cls_or_func, type):
            # It's a class
            orig_init = cls_or_func.__init__

            @functools.wraps(orig_init)
            def new_init(self, *args, **kwargs):
                warnings.warn(
                    f"{cls_or_func.__name__} is deprecated: {reason}",
                    DeprecationWarning,
                    stacklevel=2
                )
                orig_init(self, *args, **kwargs)

            cls_or_func.__init__ = new_init
            return cls_or_func
        else:
            # It's a function
            @functools.wraps(cls_or_func)
            def wrapper(*args, **kwargs):
                warnings.warn(
                    f"{cls_or_func.__name__} is deprecated: {reason}",
                    DeprecationWarning,
                    stacklevel=2
                )
                return cls_or_func(*args, **kwargs)
            return wrapper
    return decorator

def log_command(cls=None, *, console=True, nt=False, print_init=True, print_end=True):
    """
    A class decorator that adds logging to the initialize and end methods of a command.
    Added to 2429 code in the off season of 2025 to make our commands simpler to read

    Usage:
        @log_command
        class MyCommand(Command): ...

        @log_command(console=True, nt=True)
        class MyCommand(Command): ...

    How it works:
    This function handles both usage styles by checking the 'cls' argument.
    1. @log_command: Python passes the class as the first argument ('cls').
       We process and return the class immediately.
    2. @log_command(...): Python calls this function first (cls is None).
       We return the '_process_class' function. Python then calls that
       returned function with the class.

    Args:
        console (bool): Whether to print to the console (default True).
        nt (bool): Whether to publish to SmartDashboard 'alert' (default False).
        print_init (bool): Whether to log the start message (default True).
        print_end (bool): Whether to log the end message (default True).
    """

    def _process_class(cls_inner):
        # Capture original methods
        orig_init = getattr(cls_inner, "initialize", lambda self: None)
        orig_end = getattr(cls_inner, "end", lambda self, interrupted: None)

        @functools.wraps(orig_init)
        def new_initialize(self):
            # --- Logging Start Logic ---
            # Set start_time for duration calculation later
            if hasattr(self, 'container') and hasattr(self.container, 'timer'):
                self.start_time = round(self.container.timer.get(), 1)
            else:
                self.start_time = round(time.time(), 2)

            # --- Run Original Initialize First ---
            orig_init(self)

            if print_init:
                # Get attributes with defaults
                indent = getattr(self, "indent", 0)
                name = self.getName()

                # Check if the user wants to print extra info (e.g. target position)
                extra_info = getattr(self, "extra_log_info", "")
                if extra_info:
                    extra_info = f" with {extra_info}"
                else:
                    extra_info = ""

                # Format the message
                msg = f"{'    ' * indent}** Started {name}{extra_info} at {self.start_time:.1f} s **"

                if console:
                    print(msg, flush=True)
                if nt:
                    alert_pub.set(msg)

        @functools.wraps(orig_end)
        def new_end(self, interrupted: bool):
            # --- Run Original End ---
            orig_end(self, interrupted)

            # --- Logging End Logic ---
            if print_end:
                if hasattr(self, 'container') and hasattr(self.container, 'timer'):
                    end_time = round(self.container.timer.get(), 1)
                else:
                    end_time = time.time()

                start_time = getattr(self, "start_time", end_time)
                duration = end_time - start_time

                message = 'Interrupted' if interrupted else 'Ended'
                indent = getattr(self, "indent", 0)
                name = self.getName()

                msg = f"{'    ' * indent}** {message} {name} at {end_time:.1f} s after {duration:.1f} s **"

                if console:
                    print(msg)
                if nt:
                    alert_pub.set(msg)

        cls_inner.initialize = new_initialize
        cls_inner.end = new_end
        return cls_inner

    # Handle usage
    if cls is None:
        # Case: @log_command(console=...)
        # 'cls' is None because we were called as a function.
        # Return the wrapper function; Python will pass the class to it next.
        return _process_class
    elif isinstance(cls, type):
        # Case: @log_command (no parentheses)
        # 'cls' is the actual class. Process it immediately and return the modified class.
        return _process_class(cls)
    else:
        # Case: @log_command(True) - User passed a positional arg which got assigned to cls
        raise TypeError(
            "Configuration arguments for @log_command must be passed as keywords (e.g. @log_command(console=True))")