%pancake_150.ply

PlaceObject('newroboticstable.ply', [0,0,-0.0844]);


%% Creating the IRB ee Spatula 

    %SpatulaClass is a class and () indicates how many spatulas we want made
create_spatula = SpatulaClass(1);

    %SpatulaSpawn is a function we made in SpatulaClass to create and spawn
    %the spatula at an x,y,theta coordinate   
create_spatula.SpatulaSpawn(1,-0.7,0.5,0);

