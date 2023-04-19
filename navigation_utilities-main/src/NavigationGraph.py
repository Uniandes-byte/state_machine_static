#!/usr/bin/env python3
import networkx as nx
import ConsoleFormatter

class NavigationGraph:
    """
    Class that represents an undirected Graph whose nodes are places with his coordinates and the edges
    represents that a couple of places are connected.

    Attributes:
        consoleFormatter (ConsoleFormatter): Instance of the ConsoleFormatter class.
        graph (nx.Graph): Object of networkx that represents an undirected graph.
    """
    
    def __init__(self, pathPlaces, pathEdges):
        self.consoleFormatter=ConsoleFormatter.ConsoleFormatter()
        self.graph = self.createGraph(pathPlaces, pathEdges)   

    def createGraph(self, pathPlaces, pathEdges):
        """
        Creates an undirected graph with the nodes and edges especified in places.txt and edges.txt, respectively.

        Args:
            pathPlaces (str): Path of the file that describes each place and its corresponding coordinates.
            pathEdges (str): Path of the file that describes each edge.

        Returns:
            Returns an undirected graph (nx.Graph) with the nodes and edges especified in places.txt and edges.txt.
        """
        graph = nx.Graph()

        if self.checkPlacesFile(pathPlaces):
            placesFile = open(pathPlaces, 'r')
            for place in placesFile:
                infoPlace = place.split(',')
                graph.add_node(infoPlace[0].strip().lower(), x=float(infoPlace[1].strip()), y=float(infoPlace[2].strip()), theta=float(infoPlace[3].strip()), known=eval(infoPlace[4].strip()))
            placesFile.close()
        else:
            print(self.consoleFormatter.format("The graph could not be created", "FAIL"))
            return None

        if self.checkEdgesFile(pathEdges, graph):
            edgesFile = open(pathEdges, 'r')
            for edge in edgesFile:
                infoEdge = edge.split(',')
                graph.add_edge(infoEdge[0].strip().lower(), infoEdge[1].strip().lower())
            edgesFile.close()
        else:
            print(self.consoleFormatter.format("The graph could not be created", "FAIL"))
            return None
        print(self.consoleFormatter.format("The graph was created correctly!", "OKGREEN"))
        return graph

    def checkPlacesFile(self, pathPlaces):
        """
        Check if places file has a proper format: (name, x, y, theta, known).

        Args:
            pathPlaces (str): Path of the file that describes each place and its corresponding coordinates.

        Returns:
            Returns True if the file in the specified path has a proper format, False otherwise.
        """
        itsOk = True
        try:
            placesFile = open(pathPlaces, 'r')
        except:
            print(self.consoleFormatter.format("Could not open file "+pathPlaces, "FAIL"))
            itsOk = False
            return itsOk
        line=1
        for place in placesFile:
            infoPlace = place.split(',')
            if len(infoPlace) != 5:
                print(self.consoleFormatter.format("ERROR: Line "+str(line)+" does not have the correct number of elements. Expected: (name, x, y, theta, known)", "FAIL"))
                itsOk = False
                return itsOk
            try:
                float(infoPlace[1].strip())
                float(infoPlace[2].strip())
                float(infoPlace[3].strip())
                bool(infoPlace[4].strip())
            except:
                print(self.consoleFormatter.format("ERROR: Line "+str(line)+" does not have the correct format. Expected: (str, float, float, float, bool)", "FAIL"))
                itsOk = False
                return itsOk
            line+=1
        placesFile.close()
        return itsOk

    def checkEdgesFile(self, pathEdges, graph):
        """
        Check if edges file has a proper format (name, name) and if it is consistent with the places file.

        Args:
            pathEdges (str): Path of the file that describes each edge.
            graph (nx.Graph): Graph that was created with the places file.

        Returns:
            Returns True if the file in the specified path has a proper format and its consistent, False otherwise.
        """
        itsOk = True
        try:
            edgesFile = open(pathEdges, 'r')
        except:
            print(self.consoleFormatter.format("Could not open file "+pathEdges, "FAIL"))
            itsOk = False
            return itsOk
        line=1
        for edge in edgesFile:
            infoEdge = edge.split(',')
            if len(infoEdge) != 2:
                print(self.consoleFormatter.format("ERROR: Line "+str(line)+" does not have the correct number of elements. Expected: (name, name)", "FAIL"))
                itsOk = False
                return itsOk
            if infoEdge[0].strip().lower() not in list(graph.nodes):
                print(self.consoleFormatter.format(infoEdge[0].strip()+" in line "+str(line)+" is not a known place", "FAIL"))
                itsOk = False
                return itsOk
            if infoEdge[1].strip().lower() not in list(graph.nodes):
                print(self.consoleFormatter.format(infoEdge[1].strip()+" in line "+str(line)+" is not a known place", "FAIL"))
                itsOk = False
                return itsOk
            line+=1
        edgesFile.close()
        return itsOk