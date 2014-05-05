A middleware that reads SUMO floating car data output and processes it in PostGIS for ray tracing.

The code expects the shapefile data in PostgresSQL to be using SRID 4326 (WGS84).
Some geographic functions in the code assume the city of Porto as the location, with regards to latitude/longitude. 

For the cell maps, the unit of measure was one WGS84 second.