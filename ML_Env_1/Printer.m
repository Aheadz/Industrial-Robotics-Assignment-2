function pnt = Printer(Location_Printer)

    % Create and place the objects (assuming PlaceObject is a function that loads and places 3D objects)
    P_Shell = PlaceObject('PrintLink0.ply', Location_Printer);
    P_BedMover = PlaceObject('PrintLink1.ply', Location_Printer);
    P_Bed = PlaceObject('PrintLink2.ply', Location_Printer);
    P_Printer = PlaceObject('PrintLink3.ply', Location_Printer);

end
