
    imu_filtAX(:,1) = Ax(1);
    imu_filtAY(:,1) = Ay(1);
    imu_filtAZ(:,1) = Az(1);
    imu_filtGX(:,1) = Gx(1);
    imu_filtGY(:,1) = Gy(1);
    imu_filtGZ(:,1) = Gz(1);
    
    imu_filt2AX(:,1) = 0;
    imu_filt2AY(:,1) = 0;
    imu_filt2AZ(:,1) = Az(1);
    imu_filt2GX(:,1) = 0;
    imu_filt2GY(:,1) = 0;
    imu_filt2GZ(:,1) = 0;
alpha_=0.0475;

for i=2:length(Ax)
  imu_rawAX(:,i)=Ax(i);
  imu_rawAY(:,i)=Ay(i);
  imu_rawAZ(:,i)=Az(i);
  imu_rawGX(:,i)=Gx(i);
  imu_rawGY(:,i)=Gy(i);
  imu_rawGZ(:,i)=Gz(i);

    imu_filtAX(:,i) = alpha_ * imu_rawAX(:,i) + (1-alpha_) *  imu_filtAX(:,i-1);
    imu_filtAY(:,i) = alpha_ * imu_rawAY(:,i) + (1-alpha_) *  imu_filtAY(:,i-1);
    imu_filtAZ(:,i) = alpha_ * imu_rawAZ(:,i) + (1-alpha_) *  imu_filtAZ(:,i-1);
    imu_filtGX(:,i) = alpha_ * imu_rawGX(:,i) + (1-alpha_) *  imu_filtGX(:,i-1);
    imu_filtGY(:,i) = alpha_ * imu_rawGY(:,i) + (1-alpha_) *  imu_filtGY(:,i-1);
    imu_filtGZ(:,i) = alpha_ * imu_rawGZ(:,i) + (1-alpha_) *  imu_filtGZ(:,i-1);

    imu_filt2AX(:,i) = alpha_ * imu_filtAX(:,i) + (1-alpha_) *  imu_filt2AX(:,i-1);
    imu_filt2AY(:,i) = alpha_ * imu_filtAY(:,i) + (1-alpha_) *  imu_filt2AY(:,i-1);
    imu_filt2AZ(:,i) = alpha_ * imu_filtAZ(:,i) + (1-alpha_) *  imu_filt2AZ(:,i-1);
    imu_filt2GX(:,i) = alpha_ * imu_filtGX(:,i) + (1-alpha_) *  imu_filt2GX(:,i-1);
    imu_filt2GY(:,i) = alpha_ * imu_filtGY(:,i) + (1-alpha_) *  imu_filt2GY(:,i-1);
    imu_filt2GZ(:,i) = alpha_ * imu_filtGZ(:,i) + (1-alpha_) *  imu_filt2GZ(:,i-1);
  
    imu_filt3AX(:,i) = 2* imu_filtAX(:,i) - imu_filt2AX(:,i);
    imu_filt3AY(:,i) = 2* imu_filtAY(:,i) - imu_filt2AY(:,i);
    imu_filt3AZ(:,i) = 2* imu_filtAZ(:,i) - imu_filt2AZ(:,i);
    imu_filt3GX(:,i) = 2* imu_filtGX(:,i) - imu_filt2GX(:,i);
    imu_filt3GY(:,i) = 2* imu_filtGY(:,i) - imu_filt2GY(:,i);
    imu_filt3GZ(:,i) = 2* imu_filtGZ(:,i) - imu_filt2GZ(:,i);
  
end
    Ax=imu_filt3AX;
    Ay=imu_filt3AY;
    Az=imu_filt3AZ;

    Gx=imu_filt3GX;
    Gy=imu_filt3GY;
    Gz=imu_filt3GZ;
