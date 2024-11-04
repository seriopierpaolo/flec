#include <flec/datastructure.h>
#include <flec/utils.h>
#include <flec/ceresOptimization.h>

#include <geometry_msgs/Pose.h>


    Optimization_Result performOptimization(TfBatch* batch) {

        bool useCallbackFlag = 1;
        ceresOptimization solver(*batch, useCallbackFlag);

        Optimization_Result optResult = solver.solve();
        return optResult;

    }



    bool checkJacobianSVD(TfBatch* batch) {
        
        
         bool useCallbackFlag = 0;
        ceresOptimization solver(*batch, useCallbackFlag);

        Optimization_Result optResult = solver.solve();
        
        if (optResult.svd.minCoeff() > 5){
            return true;
        }
        else 
            return false;
        
        
        return false;
        
    }



TfAccumulator::TfAccumulator(ros::NodeHandle nh)
{
    this->transformPublisher = nh.advertise<geometry_msgs::Pose>("coarse_estimation",1); 

};



void TfAccumulator::publishTransform(Optimization_Result oR)
{
    geometry_msgs::Pose msg;

    // Set the position
    msg.position.x = oR.t.x();
    msg.position.y = oR.t.y();
    msg.position.z = oR.t.z();

    // Set the orientation (as a quaternion)
    msg.orientation.x = oR.q.x();
    msg.orientation.y = oR.q.y();
    msg.orientation.z = oR.q.z();
    msg.orientation.w = oR.q.w();

    // Publish the message
    this->transformPublisher.publish(msg);
}


void TfAccumulator::updateBatch() 
{
    int pair_time = this->pair.header_F.stamp.sec;

    if (this->batch.currentBatch.empty()) {

        
        this->batch.batch_start_time = pair_time;
        //this->batch.currentBatch.push_back(this->pair);
        
      }
    
    if (pair_time - this->batch.batch_start_time > 30) {
        if (checkJacobianSVD(&this->batch)) {
            updateBuffer();
            this->batch.batch_start_time = pair_time;
            
            // Update the tfBuffer
        } else {
            // this->batch.currentBatch.push_back(this->pair);

            //CLEAN THE BATCH
            this->batch = TfBatch();
            this->batch.batch_start_time = pair_time;
            
            //this->batch.currentBatch.push_back(this->pair);
            
            
            
            // this->pair = TfPair();
        }
             
}


        this->batch.currentBatch.push_back(pair);
        //std::cout<<pair_time - this->batch.batch_start_time<<std::endl;
        //std::cout<< this->batch.currentBatch.back().transformation_F.translation() <<std::endl;
        //std::cout<< this->batch.currentBatch.size() <<std::endl;
        this->pair = TfPair();
        

}




void TfAccumulator::updateBuffer(){
    
    for (int i = 0; i < this->batch.currentBatch.size(); i++){
        //std::cout<<"i"<<std::endl;
        this->macroBatch.currentBatch.push_back(this->batch.currentBatch[i]);
    }
    this->segmentBuffer.push_back(this->batch);
    this->batch = TfBatch();

    Optimization_Result result = performOptimization(&this->macroBatch);

    publishTransform(result);
    //std::cout << this->segmentBuffer.size() << std::endl;
    
}






