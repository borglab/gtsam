function datafile = findExampleDataFile(datasetName)
%FINDEXAMPLEDATAFILE Find a dataset in the examples folder

[ myPath, ~, ~ ] = fileparts(mfilename('fullpath'));
searchPath = { ...
    fullfile(myPath, [ '../../examples/Data/' datasetName ]) ...
    fullfile(myPath, [ '../gtsam_examples/Data/' datasetName ]) };
datafile = [];
for path = searchPath
    if exist(path{:}, 'file')
        datafile = path{:};
    end
end
if isempty(datafile)
    error([ 'Could not find example data file ' datasetName ]);
end

end

