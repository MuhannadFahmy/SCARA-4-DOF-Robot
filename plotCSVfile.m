function plotCSVfile(filename)
    dataFromCSVfile = load(filename);
    for i = 1:6
        figure(i)
        plotJoints(dataFromCSVfile,i)
    end
end