#include "timer.hpp"
#include "read_map.hpp"
#include <jps3d/planner/graph_search_2d_util.h>

#define VISUALIZE 1
#if VISUALIZE
#include <vtkSmartPointer.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkJPEGWriter.h> 
#endif

using namespace JPS;

int main(int argc, char ** argv){
  if(argc != 2) {
    printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  // Read the map from yaml
  MapReader<Vec3i, Vec3f> reader(argv[1], true); // Map read from a given file
  if(!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET, argv[1]);
    return -1;
  }

  // store map in map_util
  std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());
  //map_util->dilate(0.1, 0.1);
  //map_util->dilating();

  const Vec3f start(reader.start(0), reader.start(1), reader.start(2));
  const Vec3f goal(reader.goal(0), reader.goal(1), reader.goal(2));

  std::unique_ptr<GraphSearch2DUtil> planner_jps(new GraphSearch2DUtil(false)); // Declare a planner
  planner_jps->setMapUtil(map_util); // Set collision checking function

  std::unique_ptr<GraphSearch2DUtil> planner_astar(new GraphSearch2DUtil(false)); // Declare a planner
  planner_astar->setMapUtil(map_util); // Set collision checking function

  Timer time_jps(true);
  bool valid_jps = planner_jps->plan(start, goal, 1, true); // Plan from start to goal using JPS
  double dt_jps = time_jps.Elapsed().count();
  printf("JPS Planner takes: %f ms\n", dt_jps);
  printf("JPS Path Distance: %f\n", total_distance3f(planner_jps->getRawPath()));
  Timer time_astar(true);
  bool valid_astar = planner_astar->plan(start, goal, 1, false); // Plan from start to goal using A*
  double dt_astar = time_astar.Elapsed().count();
  printf("AStar Planner takes: %f ms\n", dt_astar);
  printf("AStar Path Distance: %f\n", total_distance3f(planner_astar->getRawPath()));

#if VISUALIZE
  const Vec3i dimI = map_util->getDim();
  const Vec3i startI = map_util->floatToInt(start);
  const Vec3i goalI= map_util->floatToInt(goal);
  std::string outputFilename = "output.jpg"; 
  // Create a 100x100 image to save into the jpeg file
  int extent[6] = { 0, dimI(0), 0, dimI(1), 0, 0 };
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource =
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource->SetExtent( extent );
  imageSource->SetScalarTypeToUnsignedChar(); // vtkJPEGWriter only accepts unsigned char input
  imageSource->SetNumberOfScalarComponents( 3 ); // 3 color channels: Red, Green and Blue

  // Fill the whole image with a blue background
  imageSource->SetDrawColor( 255, 255, 255 );
  imageSource->FillBox( extent[0], extent[1], extent[2], extent[3] );

  imageSource->SetDrawColor(127.0, 127.0, 127.0);
  for(int x = 0; x < dimI(0); x ++) {
    for(int y = 0; y < dimI(1); y ++) {
      if(!map_util->isFree(Vec3i(x, y, 0))) {
        imageSource->DrawPoint(x, y);
      }
    }
  }

  imageSource->SetDrawColor(0.0, 127.0, 255.0);
  imageSource->DrawCircle(startI[0], startI[1], 5);

  imageSource->SetDrawColor(255.0, 0.0, 0.0);
  imageSource->DrawCircle(goalI[0], goalI[1], 5);

  // Draw path from jps as black
  if(valid_jps) {
    vec_Vec3f path = planner_jps->getRawPath();
    imageSource->SetDrawColor(0.0, 0.0, 0.0);
    for(int i = 0; i < (int) path.size() - 1; i ++) {
      const Vec3i p1 = map_util->floatToInt(path[i]);
      const Vec3i p2 = map_util->floatToInt(path[i+1]);
      imageSource->FillTube(p1[0], p1[1], p2[0], p2[1], 2);
    }
 }

  // Draw path from A* as green
  if(valid_astar) {
    vec_Vec3f path = planner_astar->getRawPath();
    imageSource->SetDrawColor(0.0, 200.0, 0.0);
    for(int i = 0; i < (int) path.size() - 1; i ++) {
      const Vec3i p1 = map_util->floatToInt(path[i]);
      const Vec3i p2 = map_util->floatToInt(path[i+1]);
      imageSource->FillTube(p1[0], p1[1], p2[0], p2[1], 2);
    }
  }

#endif


  vtkSmartPointer<vtkJPEGWriter> writer = vtkSmartPointer<vtkJPEGWriter>::New();  writer->SetFileName( outputFilename.c_str() );  writer->SetInputConnection( imageSource->GetOutputPort() );  writer->Write();

  return 0;
}
