class Unchained:
    """
    Simple class with NO method chaining.
    Methods perform actions but do NOT return self.
    """
    def __init__(self, name):
        self.name = name

    def print(self, msg):
        # Action only — no return value
        print(f"{self.name} asked to print: {msg}")
        # Implicit return None


class Chained:
    """
    Simple class that SUPPORTS method chaining.
    Methods perform actions AND return self (fluent interface).
    """
    def __init__(self, name):
        self.name = name

    def print(self, msg):
        # Perform the action
        print(f"{self.name} asked to print: {msg}")
        # Return self so additional method calls can chain
        return self

# using a normal class
u = Unchained("alpha")
u.print("first")
u.print("second")   # must call again on u

# now use the class with the fluent interface
c = Chained("beta")
c.print("first").print("second").print("third")