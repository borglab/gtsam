function geoid = geoid_load_file(filename)
%GEOID_LOAD_FILE  Loads geoid data from filename

  geoid.file = filename; geoid.offset = NaN; geoid.scale = NaN;
  fid = fopen(geoid.file, 'r');
  if fid == -1, error(['Cannot open ', geoid.file]), end
  for i = 1:16
    if isfinite(geoid.offset) && isfinite(geoid.scale), break, end
    text = fgetl(fid);
    if ~ischar(text), continue, end
    text = strsplit(text, ' ');
    if length(text) == 3
      if strcmp(text{2}, 'Offset')
        geoid.offset = str2double(text{3});
      elseif strcmp(text{2}, 'Scale')
        geoid.scale = str2double(text{3});
      end
    elseif length(text) == 2
      break
    end
  end
  fclose(fid);
  if ~isfinite(geoid.offset), error('Cannot find offset'), end
  if ~isfinite(geoid.scale), error('Cannot find scale'), end
  geoid.im = imread(geoid.file);
  if ~isa(geoid.im, 'uint16'), error('Wrong image type'), end
  geoid.h = size(geoid.im, 1);
  geoid.w = size(geoid.im, 2);
  if ~(bitand(geoid.w, 1) == 0 && bitand(geoid.h, 1) == 1)
    error('Image width/height not even/odd')
  end
end
