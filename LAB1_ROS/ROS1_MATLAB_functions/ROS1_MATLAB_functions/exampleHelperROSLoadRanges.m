function scanPubmsg = exampleHelperROSLoadRanges()
    %exampleHelperROSLoadRanges - Loads data for the laser scan message in startExamples
    %   
    %   See also exampleHelperROSCreateSampleNetwork
    
    %   Copyright 2014-2015 The MathWorks, Inc.
    
    laserfile = 'laserdata.mat';
    lasercell = load(laserfile);
    scanPubmsg = lasercell.laserdata;
end