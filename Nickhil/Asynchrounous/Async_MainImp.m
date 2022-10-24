    


    %move person one step, -0.05 in y direction
person = PlaceObject('person_async.ply', [person_coords(1,1),person_coords(1,2) - 0.05,person_coords(1,3)]);

    %update person_coords as it moves
person_coords = [person_coords(1,1),person_coords(1,2) - 0.05,person_coords(1,3)];

    %distance between person and laser
x_dist = person_coords(1,1) - laser_origin(1,1)
y_dist = person_coords(1,2) - laser_origin(1,2)

    %checks if person is passing laser
if  (0 < x_dist) && (x_dist < 2) && (-0.05 < y_dist) && (y_dist < 0.05)
         
    while(1)
        disp('LEAVE THE AREA')
    end
end
     %wait and then delete old person
    pause(0.1);
    delete(person);