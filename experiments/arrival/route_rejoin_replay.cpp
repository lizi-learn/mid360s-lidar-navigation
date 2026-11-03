// Recorded inputs are supplied locally; no maps are embedded in this source.
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include "p2_nav_controller/route_rejoin.hpp"
#include "p2_nav_controller/filled_footprint.hpp"
using Sample=std::array<double,3>;
int main(int argc,char **argv) {
 if(argc!=8)return 2;
 std::ifstream in(argv[1]);unsigned w,h;double r,ox,oy;in>>w>>h>>r>>ox>>oy;
 nav2_costmap_2d::Costmap2D map(w,h,r,ox,oy,0);
 for(unsigned i=0;i<w*h;++i){int c;if(!(in>>c))return 3;map.getCharMap()[i]=c;}
 std::ifstream csv(argv[2]);std::string line;std::getline(csv,line);std::vector<Sample> route;
 while(std::getline(csv,line)){std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);Sample s;if(row>>s[0]>>s[1]>>s[2])route.push_back(s);}
 Sample start{std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5])};
 std::vector<geometry_msgs::msg::Point> body;
 for(auto xy:std::vector<std::array<double,2>>{{.275,.275},{.275,-.275},{-.275,-.275},{-.275,.275}}){geometry_msgs::msg::Point p;p.x=xy[0];p.y=xy[1];body.push_back(p);}
 auto clear=[&](const std::vector<Sample>&path){
  for(size_t i=0;i<path.size();++i){const auto&a=path[i?i-1:i],&b=path[i];double da=std::remainder(b[2]-a[2],2*std::acos(-1.));
   int n=std::max({1,int(std::ceil(std::hypot(b[0]-a[0],b[1]-a[1])/.01)),int(std::ceil(std::abs(da)/.01))});
   for(int k=0;k<=n;++k){double f=double(k)/n;if(!p2_nav_controller::filled_footprint_clear(&map,body,a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]),a[2]+f*da))return false;}
  }return !path.empty();};
 const bool legacy=std::string(argv[6])=="legacy";
 std::vector<Sample> result;
 if(legacy){
  size_t nearest=0;double best=1e30;
  for(size_t i=0;i<route.size();++i){double d=std::hypot(route[i][0]-start[0],route[i][1]-start[1]);if(d<best){best=d;nearest=i;}}
  if(best<.5){size_t end=nearest;while(end+1<route.size()&&std::hypot(route[end+1][0]-start[0],route[end+1][1]-start[1])<.65)++end;
   for(size_t n=end+1;n>nearest;--n){size_t i=n-1;auto joined=p2_nav_controller::terminal_path(start,route[i],1.);joined.insert(joined.end(),route.begin()+i+1,route.end());if(clear(joined)){result=joined;break;}}
  }
 }else result=p2_nav_controller::route_rejoin(start,route,2.,clear,std::chrono::steady_clock::now()+std::chrono::seconds(2));
 double first_turn=0.,distance=0.,absolute_turn=0.;size_t end=0;
 for(size_t i=1;i<result.size();++i){auto a=result[i-1],b=result[i];double d=std::hypot(b[0]-a[0],b[1]-a[1]);if(distance==0.&&d<1e-6)first_turn+=std::remainder(b[2]-a[2],2*std::acos(-1.));distance+=d;if(distance<2.)absolute_turn+=std::abs(std::remainder(b[2]-a[2],2*std::acos(-1.)));if(end==0&&d>1e-6)end=i;}
 std::ofstream out(argv[7]);out<<"x,y,yaw\n";for(auto p:result)out<<std::setprecision(14)<<p[0]<<','<<p[1]<<','<<p[2]<<'\n';
 std::cout<<std::setprecision(14)<<"{\"planned\":"<<(!result.empty()?"true":"false")<<",\"first_pivot_rad\":"<<first_turn<<",\"two_metre_absolute_turn\":"<<absolute_turn<<",\"points\":"<<result.size()<<",\"full_sweep_clear\":"<<(clear(result)?"true":"false")<<"}\n";
 return 0;
}
