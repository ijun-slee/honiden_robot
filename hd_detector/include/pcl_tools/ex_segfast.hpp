#include <iomanip>
#include <sys/time.h>

  timeval g_tick(){
     struct timeval tv;
     gettimeofday(&tv, NULL);
     return tv;
  }

  double g_tock(timeval tprev)
  {
     struct timeval tv;
     gettimeofday(&tv, NULL);
     return (double)(tv.tv_sec-tprev.tv_sec) + (tv.tv_usec-tprev.tv_usec)/1000000.0;
  }

#include "segfast.h"


void printAll( string s1, string s2, vector<int> &data){
    int maxsize=0;
    for(uint i=0;i<data.size();i++)
        if(data[i]>maxsize) maxsize=data[i];
    std::vector<int> ocounts(maxsize+1,0);
    for(uint i=0;i<data.size();i++)
        ocounts[data[i]]++;
    for(uint i=0;i<ocounts.size();i++)
        if(ocounts[i])
                std::cout<<setw(6)<<ocounts[i]<<" "<<s1<<i<<" "<<s2<<std::endl;
}


void miniHist(int indmax, vector<int> bounds, string s1, string s2, vector<int> &data){
        std::vector<int> counts(indmax+bounds.size()+2,0);
        bounds.push_back(INT_MAX); //this makes it easier for assigning...
    for(uint i=0;i<data.size();i++)
        if(data[i]<=indmax)
                counts[data[i]]++;
        else{
                for(uint j=0;j<bounds.size();j++) //data[i] is guaranteed to be w/in bounds
                        if(data[i]<=bounds[j]){
                                counts[indmax+j+1]++;
                                break;
                        }
        }
    //now print out the summary:
    //for the individual counts:
    for(uint i=0;i<=indmax;i++)
        if(counts[i])
                std::cout<<setw(6)<<counts[i]<<" "<<s1<<setw(4)<<i<<" "<<s2<<std::endl;
    //between indmax and the first bounds
        if(counts[indmax+1])
                std::cout<<setw(6)<<counts[indmax+1]<<" "<<s1<<setw(4)<<" between "<<setw(4)<<indmax+1<<" and "<<setw(4)<<bounds[0]<<" "<<s2<<std::endl;
        //between the given bounds
        for(uint i=0;i<bounds.size()-2;i++)
                if(counts[indmax+2+i])
                        std::cout<<setw(6)<<counts[indmax+2+i]<<" "<<s1<<setw(4)<<" between "<<setw(4)<<bounds[i]+1<<" and "<<setw(4)<<bounds[i+1]<<" "<<s2<<std::endl;
        //greater than the last bound
        bounds.pop_back();
        if(counts.back())
                std::cout<<setw(6)<<counts.back()<<" "<<s1<<setw(4)<<" more than "<<setw(4)<<bounds.back()+1<<" "<<s2<<std::endl;

}

//int analyzePairings(std::vector< std::vector<int> > &pairings, std::vector<int> &clustering){
//      clustering.resize(pairings.size(),-1);
//      int clusters=0;
//      std::vector<int> tosee,seen;
//      int current;
//              for(int i=pairings.size()-1; i >=0; i--){
//              if(clustering[i]!=-1) continue;
//              if(pairings[i].size()==0){//it saw no marked pts
//                      clustering[i]=clusters++;
//                      continue;
//              }
//              seen.clear();
//              seen.push_back(i);
//              tosee=pairings[i];
//              while(tosee.size()){
//                      current=tosee.back();
//                      tosee.pop_back();
//                      if(!count(seen.begin(),seen.end(),current)){
//                              seen.push_back(current);
//                              //see if we just discovered a previously explored tree:
//                              if(clustering[current] != -1){ //yes, we did!
//                                      //this tree is fully explored, so send all the nodes to the seen
//                                      for(uint j=0;j<clustering.size();j++)
//                                              if(clustering[j]==clustering[current])
//                                                      seen.push_back(j);
//                              }
//                              else
//                                      if(pairings[current].size())
//                                              tosee.insert(tosee.end(),pairings[current].begin(),pairings[current].end());
//
//                      }
//              }
//              //now have a fully explored tree in seen
//              for(uint j=0;j<seen.size();j++)
//                      clustering[seen[j]]=clusters;
//              clusters++;
//
//
//
//      }
//      return clusters;
//
//}

//
//int main(int argc, char **argv) {
//      if(argc<3){
//              std::cout<<"Usage: filename.pcd <tolerance>"<<std::endl;
//              return -1;
//      }
//
//      pcl::PointCloud<pcl::PointXYZ> cloud;
//    pcl::io::loadPCDFile(argv[1],cloud);
//    double cluster_tol=atof(argv[2]);
//
//
//
//
//
//    //---------------Count overlaps----------------------------
//
//    //---------------Count cluster size----------------------------
//
//
//    //---------------Count cluster initial size----------------------------
//
//    //---------------count pairings----------------------------
//
//
//
//
//
//  //  int totalsize=0;
//        vector<int> csize(pmap4.clusters.size());
//        for(uint i=0;i<pmap4.clusters.size();i++){
//              csize[i]=pmap4.clusters[i].size();
//  //          totalsize+=csize[i];
//        }
//  //      cout<<totalsize<<endl;
//      miniHist(10,bounds,"clusters have ","heads",csize);
//
//
//
//}


//currently testing with:
//testlogs/testcloud05_1.0.pcd .07


