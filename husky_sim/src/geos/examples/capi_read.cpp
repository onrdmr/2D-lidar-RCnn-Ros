/*
 * # GEOS C API example 1
 *
 * Reads two WKT representations and calculates the
 * intersection, prints it out, and cleans up.
 */

/* To print to stdout */
#include <stdio.h>
#include <stdarg.h>

/* Only the CAPI header is required */
#include <geos_c.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>

template <typename Out>
void split(const std::string& s, char delim, Out result)
{
  std::istringstream iss(s);
  std::string item;
  while (std::getline(iss, item, delim))
  {
    *result++ = item;
  }
}

std::vector<std::string> split(const std::string& s, char delim)
{
  std::vector<std::string> elems;
  split(s, delim, std::back_inserter(elems));
  return elems;
}
static void geos_message_handler(const char* fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vprintf(fmt, ap);
  va_end(ap);
}

int main()
{
  /* Send notice and error messages to our stdout handler */
  initGEOS(geos_message_handler, geos_message_handler);

  //   char wkb_a[100];
  //   wkb_ais.read(wkb_a, 100);

  //   int gcount = wkb_ais.gcount();

  //   printf("%s %d", wkb_a, gcount);
  //   printf("%s\n", deneme);
  unsigned char buffer_a[300000];

  FILE* filp_a = fopen("/home/onur/building_editor_models/wall1/mmap/line_0", "rb");
  int bytes_read_a = fread(buffer_a, sizeof(unsigned char), 300000, filp_a);

  printf("buffer 1 : %d\n bytes", bytes_read_a);
  unsigned char buffer_b[300000];

  FILE* filp_b = fopen("/home/onur/building_editor_models/wall1/mmap/line_1", "rb");
  int bytes_read_b = fread(buffer_b, sizeof(unsigned char), 300000, filp_b);
  printf("buffer 2 : %d bytes \n", bytes_read_b);

  fclose(filp_a);
  fclose(filp_b);
  //   unsigned char* a;
  //   a = (unsigned char*)wkb_b;
  //   /* Read the WKT into geometry objects */
  GEOSWKBReader* reader = GEOSWKBReader_create();
  GEOSWKTReader* readerW = GEOSWKTReader_create();

  const char* polygon =
      "POLYGON((6.622028 4.459076, 5.894697 4.958233, 6.306469 4.239241, 7.033800 3.740083, 6.622028 4.459076))";

  GEOSGeometry* geom_a = GEOSWKTReader_read(readerW, polygon);
  // GEOSGeometry* geom_a = GEOSWKBReader_read(reader, buffer_a, bytes_read_a);
  GEOSGeometry* geom_b = GEOSWKBReader_read(reader, buffer_b, bytes_read_b);
  /* Calculate the intersection */
  GEOSGeometry* inter = GEOSIntersection(geom_a, geom_b);

  /* Convert result to WKT */
  GEOSWKTWriter* writer = GEOSWKTWriter_create();
  /* Trim trailing zeros off output */
  GEOSWKTWriter_setTrim(writer, 1);
  char* wkt_inter = GEOSWKTWriter_write(writer, inter);

  /* Print answer */
  // printf("Geometry A:         %s\n", wkt_a);
  // printf("Geometry B:         %s\n", wkt_b);
  printf("Intersection(A, B): %s\n", wkt_inter);

  /* Clean up everything we allocated */
  GEOSWKBReader_destroy(reader);
  GEOSWKTWriter_destroy(writer);
  GEOSGeom_destroy(geom_a);
  GEOSGeom_destroy(geom_b);
  GEOSGeom_destroy(inter);
  GEOSFree(wkt_inter);

  /* Clean up the global context */
  finishGEOS();

  /* test2 */
  float position_x;
  float position_y;
  float position_z;
  std::ifstream robotPosFile("/home/onur/building_editor_models/wall1/robot_position_1");
  robotPosFile >> position_x >> position_y >> position_z;

  std::cout << position_x << " " << position_y << " " << position_z << std::endl;
  /* Done */
  return 0;
}
