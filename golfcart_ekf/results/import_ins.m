function vectornavins = import_ins(filename)
    % IMPORTFILE Import data from a text file
    %  VECTORNAVINS = IMPORTFILE(FILENAME) reads data from text file
    %  FILENAME for the default selection.  Returns the data as a table.
    %
    %  VECTORNAVINS = IMPORTFILE(FILE, DATALINES) reads data for the
    %  specified row interval(s) of text file FILENAME. Specify DATALINES as
    %  a positive scalar integer or a N-by-2 array of positive scalar
    %  integers for dis-contiguous row intervals.
    %
    %  Example:
    %  vectornavins = importfile("/home/jacob/catkin_ws/src/golfcart-gps-odom-ekf/golfcart_ekf/results/2-vectornav-ins.csv", [1, Inf]);
    %
    %  See also READTABLE.
    %
    % Auto-generated by MATLAB on 20-Apr-2020 11:48:22

    dataLines = [2, Inf];


    %% Setup the Import Options and import the data
    opts = delimitedTextImportOptions("NumVariables", 24);

    % Specify range and delimiter
    opts.DataLines = dataLines;
    opts.Delimiter = ",";

    % Specify column names and types
    opts.VariableNames = ["Var1", "Var2", "Var3", "Var4", "Var5", "Var6", "Var7", "Var8", "Var9", "Var10", "yaw", "lat", "lon", "alt", "Var15", "Var16", "Var17", "Var18", "Var19", "Var20", "Var21", "Var22", "Var23", "Var24"];
    opts.SelectedVariableNames = ["yaw", "lon", "lat", "alt"];
    opts.VariableTypes = ["string", "string", "string", "string", "string", "string", "string", "string", "string", "string", "double", "double", "double", "double", "string", "string", "string", "string", "string", "string", "string", "string", "string", "string"];

    % Specify file level properties
    opts.ExtraColumnsRule = "ignore";
    opts.EmptyLineRule = "read";

    % Specify variable properties
    opts = setvaropts(opts, ["Var1", "Var2", "Var3", "Var4", "Var5", "Var6", "Var7", "Var8", "Var9", "Var10", "Var15", "Var16", "Var17", "Var18", "Var19", "Var20", "Var21", "Var22", "Var23", "Var24"], "WhitespaceRule", "preserve");
    opts = setvaropts(opts, ["Var1", "Var2", "Var3", "Var4", "Var5", "Var6", "Var7", "Var8", "Var9", "Var10", "Var15", "Var16", "Var17", "Var18", "Var19", "Var20", "Var21", "Var22", "Var23", "Var24"], "EmptyFieldRule", "auto");

    % Import the data
    vectornavins = readtable(filename, opts);

end