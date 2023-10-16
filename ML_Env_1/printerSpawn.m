classdef printerSpawn
    %PRINTERSPAWN used to declare a printer object in the simulation.
    properties
        P_Shell
        P_BedMover
        P_Bed
        P_Printer
    end
    
    methods
        function self = printerSpawn(Location_Printer)
            %PRINTERSPAWN Construct an instance of this class
            % Create and place the objects (assuming PlaceObject is a function that loads and places 3D objects)
            self.P_Shell = PlaceObject('PrintLink0.PLY', Location_Printer);
            self.P_BedMover = PlaceObject('PrintLink1.PLY', Location_Printer);
            self.P_Bed = PlaceObject('PrintLink2.PLY', Location_Printer);
            self.P_Printer = PlaceObject('PrintLink3.PLY', Location_Printer);
        end
        
        function outputArg = movePrintBed(self,distance)
            %TO-DO write function that animates the print bed movement.
        end
    end
end

