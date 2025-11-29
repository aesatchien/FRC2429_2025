"""
lambdas.py
-----------
Quick reference on Python lambdas, with emphasis on:
- What lambdas are (and are not)
- When you actually need a callable evaluated at runtime
- How this shows up in RobotPy / WPILib Commands2
- How to avoid lambdas using equivalent alternatives

This module is intended as *documentation-only*.
Import it in your RobotPy project so students have a local cheat sheet (but do not use it for runtime logic):

    import lambdas  # noqa: F401 (used for docs only)


============================================================
1. WHAT IS A LAMBDA?
============================================================

A lambda is just a small, anonymous function defined with:

    lambda <arguments>: <single_expression>

Examples:

    f = lambda x: x + 1
    f(3)        # -> 4

    g = lambda: 42
    g()         # -> 42

Key points:

- A lambda is a normal function object.
- The body must be a *single expression* (no multiple statements).
- The value of the expression is the return value.
- It does NOT run when it is defined. It runs when called, like any function.

You almost never “need a lambda”.
APIs need a *callable* — a function that can be called later.
You can provide that with:

    - a lambda
    - a function defined with `def`
    - a bound method, e.g. `intake.runIn`
    - a callable object (`__call__`)


============================================================
2. WHEN DO YOU NEED A CALLABLE (RUNTIME EVALUATION)?
============================================================

You need a callable whenever an API wants to:

- do something *later*, not right now, OR
- recompute a value each time it is needed.

Typical patterns:

- Callbacks (GUI, events, timers)
- **Suppliers** (functions that return a value when called)
- Predicates / conditions (functions returning bool)
- Transformers (functions applied to data items)

Generic Python examples:

    # sorted with a key callable
    data = ["robot", "FRC", "commands", "lambda"]
    by_length = sorted(data, key=lambda s: len(s))

    # filter with a predicate
    numbers = [0, 1, 2, 3, 4, 5]
    evens = list(filter(lambda x: x % 2 == 0, numbers))  # [0, 2, 4]

RobotPy / WPILib examples:

- Commands2 wants callables for:
    - things to run each scheduler loop (`RunCommand`, `FunctionalCommand`)
    - “start” and “end” callbacks (`StartEndCommand`)
    - boolean conditions (`Trigger`, `ConditionalCommand`)
- These must be evaluated at runtime, not once at construction.


============================================================
3. BASIC LAMBDA EXAMPLES (PYTHON-ONLY)
============================================================

3.1 Simple math functions:

    increment = lambda x: x + 1
    double    = lambda x: 2 * x

    increment(3)  # -> 4
    double(5)     # -> 10

3.2 Zero-argument lambdas:

    always_true  = lambda: True
    always_hello = lambda: "hello"

    always_true()   # -> True
    always_hello()  # -> "hello"

3.3 Use as a key function:

    data = ["Robot", "frc", "Commands"]
    # case-insensitive sort
    sorted_data = sorted(data, key=lambda s: s.lower())

3.4 Small closures (functions that capture context):

    def make_adder(n):
        # Return a function that adds n to its input.
        return lambda x: x + n

    add5 = make_adder(5)
    add5(10)     # -> 15

This shows that lambdas can capture variables from the surrounding scope
(e.g., n) and use them later. The same effect can be achieved with `def`.


============================================================
4. COMMON MISTAKES & MISCONCEPTIONS
============================================================

4.1 Forgetting to CALL the lambda:

    f = lambda: 42
    print(f)    # prints function object, not 42
    print(f())  # prints 42

4.2 Doing work immediately instead of passing a callable:

    # WRONG: called immediately, result (None) is passed in
    do_something_now = print("hi")   # runs now

    # CORRECT: create a function that can be called later
    do_something_later = lambda: print("hi")

4.3 Trying to put multiple statements in a lambda (not allowed):

    # INVALID:
    #    lambda x: y = x + 1
    #
    # Use def instead:

    def process(x):
        y = x + 1
        print(y)
        return y

4.4 **The Late-Binding Closure Pitfall (Advanced)**

    # WRONG: In a loop, the variable (i) is only evaluated when the
    # lambda is called. All created functions share the final value of i.
    callables = []
    for i in range(5):
        callables.append(lambda: i * 2)

    # callables[0]() will be 8 (since the loop finishes with i=4)

    # FIX: Use a default argument to force immediate capture of the value
    callables_fixed = []
    for i in range(5):
        callables_fixed.append(lambda val=i: val * 2)

    # callables_fixed[0]() is 0. callables_fixed[4]() is 8.

4.5 “Lambdas are faster.”

    No. Lambdas are not special or faster than `def`. They are just shorter.

4.6 “Lambdas run automatically.”

    No. Just like any function, they run only when called:

        h = lambda: do_something()
        h()  # required


============================================================
5. ROBOTPY / WPILIB PATTERNS WITH LAMBDAS
============================================================

The key concept: many Commands2 APIs want *callables*.

- Something to run every scheduler loop
- A condition to test repeatedly
- A one-shot action when an event occurs

If you pass a value, it’s used once now.
If you pass a callable, it can be used later, many times.


------------------------------------------------------------
5.1 Default drive command (runtime joystick reads)
------------------------------------------------------------

    from commands2 import RunCommand
    from wpilib import Joystick

    joystick = Joystick(0)      # example only
    drivetrain = ...            # your subsystem

    # WRONG (evaluates once at construction time):
    # drive_cmd = RunCommand(
    #     drivetrain.arcadeDrive(joystick.getY(), joystick.getX()),
    #     [drivetrain]
    # )

    # CORRECT (evaluates joystick each scheduler run):
    drive_cmd = RunCommand(
        lambda: drivetrain.arcadeDrive(joystick.getY(), joystick.getX()),
        [drivetrain]
    )

This is what “runtime evaluation” means in practice:
the joystick is read every time the scheduler invokes the command.


------------------------------------------------------------
5.2 Button bindings (InstantCommand, RunCommand)
------------------------------------------------------------

    from commands2.button import JoystickButton
    from commands2 import InstantCommand, RunCommand

    joystick = ... # defined earlier
    shoot_button = JoystickButton(joystick, 1)
    shooter = ...   # shooter subsystem

    # WRONG: calls spinUp() during construction:
    # shoot_button.onTrue(
    #     InstantCommand(shooter.spinUp(), [shooter])
    # )

    # CORRECT: pass a callable:
    shoot_button.onTrue(
        InstantCommand(lambda: shooter.spinUp(), [shooter])
    )

    # While held: run at a given RPM
    shoot_button.whileTrue(
        RunCommand(lambda: shooter.runAtRPM(3000), [shooter])
    )


------------------------------------------------------------
5.3 Triggers and boolean conditions
------------------------------------------------------------

    from commands2.button import Trigger

    # Suppose sensor.get() returns True if cargo present
    sensor = ...

    cargo_trigger = Trigger(lambda: sensor.get())

    cargo_trigger.onTrue(
        InstantCommand(lambda: intake.extend(), [intake])
    )
    cargo_trigger.onFalse(
        InstantCommand(lambda: intake.retract(), [intake])
    )

`lambda: sensor.get()` is a BooleanSupplier.
It is evaluated each time the trigger system checks conditions.


------------------------------------------------------------
5.4 ConditionalCommand (runtime choice between commands)
------------------------------------------------------------

    from commands2 import ConditionalCommand, RunCommand

    joystick = ... # defined earlier

    def is_turning():
        return abs(joystick.getX()) > 0.1

    turn_command = RunCommand(
        lambda: drivetrain.arcadeDrive(0.0, joystick.getX()),
        [drivetrain]
    )

    drive_straight_command = RunCommand(
        lambda: drivetrain.arcadeDrive(joystick.getY(), 0.0),
        [drivetrain]
    )

    conditional_drive = ConditionalCommand(
        onTrue=turn_command,
        onFalse=drive_straight_command,
        condition=is_turning  # or lambda: abs(joystick.getX()) > 0.1
    )

`condition` must be a callable that returns bool.
It is evaluated when the ConditionalCommand needs to decide which branch to run.


------------------------------------------------------------
5.5 StartEndCommand
------------------------------------------------------------

    from commands2 import StartEndCommand

    # Intake example: run intake while the command is active.

    intake_command = StartEndCommand(
        onStart=lambda: intake.setPower(0.8),
        onEnd=lambda interrupted: intake.setPower(0.0),
        requirements=[intake]
    )

- `onStart` is a callable taking no args.
- `onEnd` is a callable taking a single bool (interrupted flag).
- Lambdas provide compact inline definitions for these callbacks.


============================================================
6. HOW TO AVOID LAMBDAS (EQUIVALENT ALTERNATIVES)
============================================================

Lambdas are convenient, but not required. Any callable with the correct
signature is acceptable.

You can always replace a lambda with:

    - a named function (`def`)
    - a bound method (`intake.runIn`)
    - a callable object (class with `__call__`)


------------------------------------------------------------
6.1 Replace lambda with named functions
------------------------------------------------------------

Original with lambdas:

    intake_command = StartEndCommand(
        onStart=lambda: intake.setPower(0.8),
        onEnd=lambda interrupted: intake.setPower(0.0),
        requirements=[intake]
    )

Equivalent using `def`:

    def intake_start():
        intake.setPower(0.8)

    def intake_end(interrupted: bool): # Must take 'interrupted' argument
        intake.setPower(0.0)

    intake_command = StartEndCommand(
        onStart=intake_start,
        onEnd=intake_end,
        requirements=[intake]
    )


------------------------------------------------------------
6.2 Use subsystem methods directly
------------------------------------------------------------

    class Intake:
        def setPower(self, power: float) -> None:
            ...

        def runIn(self) -> None: # Signature for onStart (no args)
            self.setPower(0.8)

        # IMPORTANT: Must accept 'interrupted' argument for onEnd
        def stop(self, interrupted: bool) -> None:
            self.setPower(0.0)

    intake = Intake()

    intake_command2 = StartEndCommand(
        onStart=intake.runIn,
        onEnd=intake.stop, # Bound method used as callable
        requirements=[intake]
    )

Here, `intake.runIn` and `intake.stop` are bound methods.
They are callables that match the required signatures.


------------------------------------------------------------
6.3 Named predicates / suppliers
------------------------------------------------------------

Original with lambda:

    Trigger(lambda: sensor.get()).onTrue(...)

Equivalent using a named function:

    def cargo_present() -> bool:
        return sensor.get()

    cargo_trigger = Trigger(cargo_present)

    cargo_trigger.onTrue(
        InstantCommand(intake.runIn, [intake])
    )
    cargo_trigger.onFalse(
        InstantCommand(intake.stop, [intake])
    )


------------------------------------------------------------
6.4 Default drive command without lambdas
------------------------------------------------------------

Original with lambda:

    drive_cmd = RunCommand(
        lambda: drivetrain.arcadeDrive(joystick.getY(), joystick.getX()),
        [drivetrain]
    )

Equivalent using `def`:

    def arcade_drive_with_joystick() -> None:
        y = joystick.getY()
        x = joystick.getX()
        drivetrain.arcadeDrive(y, x)

    drive_cmd = RunCommand(arcade_drive_with_joystick, [drivetrain])


------------------------------------------------------------
6.5 Callable class (stateful callbacks)
------------------------------------------------------------

Use this when the callback needs persistent state across calls.

    class TimedIntakeStart:
        def __init__(self, subsystem, power: float):
            self.subsystem = subsystem
            self.power = power
            self.count = 0

        def __call__(self) -> None: # Signature for onStart (no args)
            self.subsystem.setPower(self.power)
            self.count += 1    # track how many times it was called

    timed_intake_start = TimedIntakeStart(intake, 0.8)

    intake_command3 = StartEndCommand(
        onStart=timed_intake_start,    # callable object
        onEnd=intake.stop,
        requirements=[intake]
    )


============================================================
7. SUMMARY FOR STUDENTS
============================================================

1) Lambdas are just anonymous functions:

        lambda args: expression

2) Commands2 and other APIs want *callables*, not specifically lambdas.
    You can provide:
    - lambdas
    - functions defined with `def`
    - methods or callable objects

3) Use a callable whenever something must be evaluated at *runtime*
    (joystick readings, sensor states, conditions).

4) If the logic is more than a simple expression, `def` is usually clearer
    than a lambda.

5) For most FRC use cases, pick whichever form (lambda vs. def) makes the
    intent most obvious and keeps side effects from happening at construction
    time instead of at runtime.

"""
# This module intentionally defines no runtime code.
# It is meant to be imported as in-project documentation.