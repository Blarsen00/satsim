import yaml

class Plot:
    def __init__(self, filename):
        with open(filename, 'r') as file:
            self.param = yaml.safe_load(file)
        file.close()

        self.name = self.param["name"]
        self.size = self.param["size"]
        self.ylim = self.param["ylim"]
        self.colors = self.param["colors"]
        self.ylables = self.param["ylabels"]
        self.grid = self.param["grid"]
        self.reference = self.param["reference"]

        print(f"#################### Plot {self.name} parameters ####################")
        print(f"- Number of plots: {self.size}")
        print(f"- Limit:")
        for i in range(self.size):
            print(f"{i+1}. Plot: {self.ylim[i % len(self.ylim)]}")
        print(f"- Axis colors: {self.colors}")
        for i in range(self.size):
            print(f"{i+1}. Plot: {self.colors[i % len(self.colors)]}")
        print(f"- Y-labels for the plot: {self.ylables}")
        print(f"- Plot grid: \n{self.grid}")
        print(f"- Plot reference: \n{self.reference}")
        print("##############################################################")

    def set_data(self, data):
        self.data = data

    def set_reference_data(self, data):
        self.reference_data = data
