class Logger:
    def __init__(self, filename="log.txt"):
        self.file = open(filename, 'a')  # open file once in append mode

    def write(self, data):
        """Write a line of text to the file"""
        self.file.write(f"{data}" + '\n')
        self.file.flush()  # ensure it’s saved immediately

    def close(self):
        """Close file when done"""
        self.file.close()
