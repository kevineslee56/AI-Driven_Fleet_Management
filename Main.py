from MapMaker import GenerateMap, PlotMap

def main():
    nodes_list = GenerateMap(5, 2, 0, 100, 0, 100)
    PlotMap(nodes_list)

if __name__ == "__main__":
    main()
