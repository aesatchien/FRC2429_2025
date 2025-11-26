"""
FRC COMMANDS2 CHEAT SHEET (ROBOTPY — 2025)

----------------------------------------------------------------------
1. TRIGGER ACTIONS (BINDINGS)

These are the actions a Trigger can take when its boolean state changes.

onTrue(cmd)
    Run once when trigger transitions False -> True.

onFalse(cmd)
    Run once when trigger transitions True -> False.

onChange(cmd)
    Run once on any state change (both edges).

whileTrue(cmd)
    Start cmd when trigger becomes True.
    Cancel cmd when trigger becomes False.

whileFalse(cmd)
    Start cmd when trigger becomes False.
    Cancel cmd when trigger becomes True.

toggleOnTrue(cmd)
    On rising edge: if cmd is not running, schedule it;
                    if cmd is running, cancel it.

toggleOnFalse(cmd)
    Same as above but triggers on falling edge.

Trigger logic combinators (used to create new Triggers):
    t1 & t2        logical AND
    t1 | t2        logical OR
    ~t1            logical NOT
    t.debounce(sec, type)   return debounced version of the Trigger


----------------------------------------------------------------------
2. COMMAND DECORATORS (WRAPPERS)

These methods create new Commands from existing ones. They allow
sequencing, parallel runs, timeouts, conditional execution, and more.

SEQUENCING
----------
andThen(next)
    Run this command, then run the next one.

beforeStarting(pre)
    Run the pre command/action first, then run this command.


PARALLEL EXECUTION
------------------
alongWith(other)
    Run both commands in parallel; finish when both finish.

raceWith(other)
    Run both in parallel; finish when either one finishes (cancel the rest).

deadlineFor(other)
    Run both in parallel; the current command is the "deadline".
    When the deadline command finishes, all others are canceled.


TIMEOUTS AND CONDITIONS
-----------------------
withTimeout(seconds)
    Cancel the command if the timeout expires.

until(condition)
    Cancel the command when the condition function returns True.

onlyWhile(condition)
    Keep the command running only while the condition is True.

repeatedly()
    When the command ends normally, restart it immediately.


CONDITIONAL EXECUTION
---------------------
onlyIf(condition)
    Run this command only if condition() is True.

unless(condition)
    Run this command only if condition() is False.


BEHAVIOR AND METADATA WRAPPERS
------------------------------
ignoringDisable(True or False)
    Allow a command to run when the robot is disabled.

withInterruptBehavior(mode)
    Control interrupt behavior: kCancelIncoming or kCancelSelf.

finallyDo(fn)
    Execute fn() after end() is called.

handleInterrupt(fn)
    Execute fn() only if the command was interrupted.

withName(name)
    Assign a custom telemetry/SmartDashboard name.

asProxy()
    Wrap the command so it doesn’t inherit group requirements.


----------------------------------------------------------------------
3. MINIMAL EXAMPLES

Trigger binding:
    trigger.onTrue(PrintCommand("pressed").ignoringDisable(True))

Command chaining:
    driveCmd.withTimeout(2).andThen(balanceCmd).ignoringDisable(True)

Parallel:
    intakeCmd.alongWith(armCmd)

Debounced trigger:
    controller.a().debounce(0.2).onTrue(FireCommand())
"""
