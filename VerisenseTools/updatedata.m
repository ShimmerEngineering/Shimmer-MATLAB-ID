function void = updatedata( filename, noise, offset, resolution, alignment)

data = importdata(filename);
if noise~=0
    xmin = -noise;
    xmax = noise;
    noise_matrix = xmin+rand(size(data.data,1),3)*(xmax-xmin);
    data.data = data.data + noise_matrix;
end

if offset~=0
    offset_matrix = zeros(size(data.data,1),3) + offset;
    data.data = data.data + offset_matrix;
end

if resolution~=0
    temp = ceil(1 / resolution);
    data.data = round(data.data * temp) / temp;
end

data.data = data.data * alignment;

% data.data(data.data > 4) = 4;
% data.data(data.data < -4) = -4;

csvwrite('combined_file.csv', data.data);

end