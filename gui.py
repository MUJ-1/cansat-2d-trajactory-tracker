import tkinter as tk

class LivePrintGUI:
    """
    A Tkinter-based GUI that displays live updates from formatted strings
    like 'ax_raw = 1.23, ay_raw = 4.56, az_raw = 9.81, gx = 0.1, gy = 0.2, gz = 0.3'.
    
    Usage:
        gui = LivePrintGUI()
        # In your loop:
        # s = f'ax_raw = {ax_raw}, ay_raw = {ay_raw}, ...'
        # gui.update(s)
        # Optionally: gui.root.update() to force GUI refresh if loop is tight
        # gui.root.mainloop()  # Run at the end to keep window open
    """
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Live Print Monitor")
        self.root.geometry("400x200")
        self.value_labels = {}  # var_name: value_label widget
        self.setup_done = False
        self.main_frame = None

    def update(self, formatted_string):
        """
        Update the GUI with the given formatted string.
        Parses the string to extract var = value pairs.
        Creates labels on first call, updates values thereafter.
        """
        # Parse the string: split by comma, then by ' = '
        parts = [part.strip() for part in formatted_string.split(',')]
        var_values = {}
        for part in parts:
            if ' = ' in part:
                var, val = part.split(' = ', 1)
                var = var.strip()
                val = val.strip()
                var_values[var] = val

        if not self.setup_done:
            self._create_labels(var_values)
            self.setup_done = True

        # Update existing value labels
        for var, val in var_values.items():
            if var in self.value_labels:
                self.value_labels[var].config(text=val)

        # Refresh the GUI without blocking (lightweight)
        self.root.update_idletasks()

    def _create_labels(self, var_values):
        """Create fixed labels and initial value displays."""
        # Clear any existing frame
        if self.main_frame:
            self.main_frame.destroy()

        self.main_frame = tk.Frame(self.root, padx=10, pady=10)
        self.main_frame.pack(fill=tk.BOTH, expand=True)

        # Configure columns for spacing (column 0 narrow, column 1 wider)
        self.main_frame.columnconfigure(0, minsize=80, weight=1)
        self.main_frame.columnconfigure(1, minsize=120, weight=1)

        # Use grid for clean alignment
        for i, var in enumerate(sorted(var_values.keys())):  # Sort for consistent order
            # Variable label (fixed)
            var_label = tk.Label(
                self.main_frame,
                text=f"{var} =",
                anchor='w',
                font=('Arial', 10),
                padx=5  # Simplified uniform padding
            )
            var_label.grid(row=i, column=0, sticky='w', pady=2)

            # Value label (updates)
            val_label = tk.Label(
                self.main_frame,
                text=var_values[var],
                anchor='w',
                relief=tk.SUNKEN,
                width=15,
                font=('Courier', 10),
                bg='white'
            )
            val_label.grid(row=i, column=1, sticky='w', pady=2)

            self.value_labels[var] = val_label

    def run(self):
        """Start the Tkinter main loop. Call this after your loop if needed."""
        self.root.mainloop()