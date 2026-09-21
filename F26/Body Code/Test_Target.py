import threading


class TargetTestUI:
    """
    Terminal-based fake beacon for testing BB-8 navigation.

    Distance: feet

    Angle:
         0 = straight ahead
        +  = target to the right
        -  = target to the left
    """

    def __init__(self):
        self.distance = None
        self.angle = None
        self.running = True

        print("\n===================================")
        print("       BB-8 TARGET SIMULATOR")
        print("===================================")
        print("Enter targets as:")
        print("    distance angle")
        print()
        print("Examples:")
        print("    3.0 0")
        print("    3.0 30")
        print("    2.0 -45")
        print()
        print("Commands:")
        print("    stop  - remove target / stop BB-8")
        print("    quit  - exit simulator")
        print("===================================\n")

        # Start input in a separate thread so it
        # doesn't block the navigation loop.
        self.input_thread = threading.Thread(
            target=self._input_loop,
            daemon=True
        )
        self.input_thread.start()

    def _input_loop(self):

        while self.running:

            try:
                command = input("Target > ").strip()

                if not command:
                    continue

                # -------------------------
                # STOP
                # -------------------------
                if command.lower() == "stop":
                    self.distance = None
                    self.angle = None

                    print("\n[TEST BEACON] Target removed")
                    print("BB-8 should STOP\n")
                    continue

                # -------------------------
                # QUIT
                # -------------------------
                if command.lower() == "quit":
                    self.distance = None
                    self.angle = None
                    self.running = False

                    print("\n[TEST BEACON] Simulator stopped\n")
                    continue

                # -------------------------
                # TARGET
                # -------------------------
                parts = command.split()

                if len(parts) != 2:
                    print(
                        "Enter: distance angle\n"
                        "Example: 3.0 -30"
                    )
                    continue

                distance = float(parts[0])
                angle = float(parts[1])

                self.distance = distance
                self.angle = angle

                print(
                    f"\n[TEST BEACON]"
                    f" Distance = {distance:.2f} ft,"
                    f" Angle = {angle:.1f}°\n"
                )

            except ValueError:
                print(
                    "ERROR: Distance and angle "
                    "must be numbers."
                )

            except EOFError:
                self.running = False

    def get_target(self):
        return self.distance, self.angle

    def update(self):
        # Kept so Target.py doesn't need to change.
        pass