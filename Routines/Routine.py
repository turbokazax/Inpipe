class Routine:
    def __init__(self):
        """Initialize the routine (setup code)."""
        pass

    def loop(self):
        """Main control loop (runs repeatedly)."""
        pass

    def run(self):
        """Run the routine until interrupted."""
        try:
            while True:
                self.loop()
        except KeyboardInterrupt:
            print("Routine stopped by user.")
            self.onStop()
    
    def onStop(self):
        """Cleanup code to run when the routine is stopped."""
        pass