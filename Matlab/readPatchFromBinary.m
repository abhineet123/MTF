clear;
% Change this line to specify the file
filename = '../../../data/datasets_MTF/TMT/nl_bookI_s3_50x50_20_25_1_basis.bin';
fileID = fopen(filename);
rows = fread(fileID,1,'int32','a');
cols = fread(fileID,1,'int32','a');
imgs = fread(fileID,'double','a');
fclose(fileID);
filename1 = '../../../data/datasets_MTF/TMT/nl_bookI_s3_50x50_1.bin';
fileID1 = fopen(filename1);
rows1 = fread(fileID1,1,'int32','a');
cols1 = fread(fileID1,1,'int32','a');
imgs1 = fread(fileID1,'double','a');
fclose(fileID1);

%% image is column wise
n_frames = numel(imgs)/rows/(cols+1);
imgs = reshape(imgs, rows, (cols+1), n_frames);

%% Check the reconstruction error
recont_err = zeros(n_frames,1);
figure;
for i=1:n_frames
    basis_matrix = imgs(:,:,i);
    basis = basis_matrix(:,1:end-1);
    mean_img = basis_matrix(:,end);
    curr_img = imgs1(:,i);
    curr_img = curr_img - mean_img;
    err_img = curr_img - basis * (basis' * curr_img);
    recont_err(i) = norm(err_img);
    subplot(3,3,1);
    imshow(reshape(imgs1(:,i), sqrt(rows), sqrt(rows))', []); title(sprintf('Image at frame %i',i));
    subplot(3,3,2);
    imshow(reshape(curr_img, sqrt(rows), sqrt(rows))', []); title(sprintf('Mean subtracted image at frame %i',i));
    subplot(3,3,3);
    imshow(reshape(err_img, sqrt(rows), sqrt(rows))', []); title(sprintf('Error Image at frame %i',i));
    % eigen basis top 3
    subplot(3,3,4);
    imshow(reshape(basis(:,1), sqrt(rows), sqrt(rows))', []); title('Basis 1');
    subplot(3,3,5);
    imshow(reshape(basis(:,2), sqrt(rows), sqrt(rows))', []); title('Basis 2');
    subplot(3,3,6);
    imshow(reshape(basis(:,3), sqrt(rows), sqrt(rows))', []); title('Basis 3');
    subplot(3,1,3);
    plot(i, recont_err(i),'bo');title('Recont error');hold on;
    fprintf('At frame %d\n',i);
    pause();
end

%% show the eigen basis and mean images
figure(1);
for i=1:10:n_frames
   % im1 = imgs(:,:,i);% eigen basis matrix at frame i
    subplot(1,4,1);
    imshow(reshape(imgs(:,1,i), sqrt(rows), sqrt(rows))', []);
    title('The first eigen basis');
    subplot(1,4,2);
    imshow(reshape(imgs(:,cols+1,i)+imgs(:,1,i), sqrt(rows), sqrt(rows))', []);
    title('The first eigen basis + mean  imag');
    subplot(1,4,3);
    imshow(reshape(imgs(:,10,i), sqrt(rows), sqrt(rows))', []);
    title('The last eigen basis');
    subplot(1,4,4);
    imshow(reshape(imgs(:,cols+1,i), sqrt(rows), sqrt(rows))', []);
    title('The mean imge');
    fprintf('At frame %d\n',i);
    pause();
end

%% SVD
% For the first 20 frames
imgs1 = reshape(imgs1, rows1, cols1);
img = imgs1(:,50:170);
mean_img = mean(img, 2);
centered_img = img - mean_img * ones(1, size(img, 2));
[U,S,~] = svd(centered_img,'econ');
B = U(:,1:10); % use 10 basis
% -0.0043852   0.0061277  -0.0065191  0.00247141 -0.00348047   0.0283596  -0.0105163   0.0232377    0.015574   0.0187584

% check the centered image
figure(2);
for i=1:10
    imdiff(:,i) = imgs1(:,i)-imgs1(:,i+1);
    %imshow(reshape(imgs1(:,i)-imgs1(:,i+1), 50,50), []);
    %pause();
end;

%% Show the image
figure;
for i=1:rows
    subplot(rows,1,i);
    imshow(reshape(imgs1(i,:),50,50)', []);
end

%%
bad_img = img(:,10) - 50*rand(2500,1);
bad_img(1:1000) = zeros(1000,1);
black_img = rand(2500,1)*255;

%% PCA with X*X'
img = imgs;
n_imgs = 120;
n_feat = 10;
train_img = zeros(2500, n_imgs);
test_img = zeros(2500, n_imgs);
for i=1:2*n_imgs
    if mod(i,2)==1
        train_img(:,int8(i/2)) = img(:,i);
    else
        test_img(:,i/2) = img(:,i);
    end
end

mean_img = mean(train_img, 2);
centered_img = train_img - mean_img * ones(1, size(train_img, 2));
[U,S,~] = svd(centered_img);
%A = centered_img * centered_img' / (size(centered_img,2)-1); % average over the number of samples
%[V,D] = eig(A);
B = U(:,1:n_feat);

% coefficients for training images
y = zeros(n_feat, n_imgs);
for i=1:n_imgs
    y(:,i) = B' * centered_img(:,i);
end

%%
init_feat = y(:,1);
ssd = zeros(24, 1);
for i = 2:n_imgs
   feat_diff = y(:,i) - init_feat;
   ssd(i) = sum(feat_diff.^2);
end
%ssd(1)=sum((eigen_patch'*black_img-init_feat).^2);
figure(1);subplot(2,1,1);
plot(ssd);
title('Sanity check SSD w/o per image normalization');
xlabel('frames');ylabel('SSD on coefficients');

%% Reconstruction error
recon_err = zeros(n_imgs,1);
centered_test_img = test_img - mean_img * ones(1, size(test_img, 2));
img_recon = zeros(2500, n_imgs);
for i = 1:n_imgs
    img_recon(:,i) = B * B' * centered_test_img(:,i);
    recon_err(i) = norm(centered_test_img(:,i) - img_recon(:,i));
end
%rand_img = black_img - mean_img;
rand_img = test_img(:,10);
rand_img(1:100)=zeros(100,1);
rand_img = rand_img - mean_img;
recon_err(end) = norm(B * B' *  rand_img - rand_img);
recon_err = recon_err/size(test_img,1);
figure(1);
subplot(2,1,1);
plot(recon_err,'b*');
title('Reconstruction error of test image');
xlabel('frames');ylabel('Reconstruction error');
%% Reconstruction error
recon_err_train = zeros(n_imgs,1);
img_recon_train = zeros(2500, n_imgs);
for i = 1:n_imgs
    img_recon_train(:,i) = B * B' * centered_img(:,i);
    recon_err_train(i) = norm(centered_img(:,i) - img_recon_train(:,i));
end
recon_err_train = recon_err_train/size(train_img,1);
figure(1);
subplot(2,1,2);
plot(recon_err_train,'ro');
title('Reconstruction error for training images');
xlabel('frames');ylabel('Reconstruction error');
%% Residue image
figure(2);
res_img = centered_img - img_recon;
subplot(1,3,1);
imshow(reshape(res_img(:,1),50,50), []);title('Error');
subplot(1,3,2);
imshow(reshape(centered_img(:,1),50,50), []);title('Original');
subplot(1,3,3);
imshow(reshape(img_recon(:,1),50,50), []);title('Recon');
%% Compare with pure SSD
init_pixel = centered_test_img(:,1);
%init_pixel = init_pixel-min(init_pixel)/(max(init_pixel)-min(init_pixel));
ssd_pixel = zeros(n_imgs, 1);
for i = 2:n_imgs
   pixel_diff = centered_test_img(:,i) - init_pixel;
   ssd_pixel(i) = sum(pixel_diff.^2);
end
rand_img = black_img - mean_img;
%rand_img = centered_test_img(:,1);
%rand_img(1:2500)=zeros(2500,1);
ssd_pixel(1)=sum((rand_img-init_pixel).^2); % replace the first image with shifted image
ssd_pixel = ssd_pixel / 2500;
figure(1);
subplot(2,1,2);
plot(ssd_pixel,'ro');
title('Sanity check SSD on pixels w/o per image normalization');
xlabel('frames');ylabel('SSD on pixels');
%diff_ssd = ssd_pixel - ssd;
%plot(diff_ssd, 'y*');

figure;
imshow(reshape(rand_img,50,50),[]);


%% Normalize each ssd
norm_ssd = recon_err / norm(recon_err);
norm_ssd_pixel = ssd_pixel / norm(ssd_pixel);
figure(3);
plot(norm_ssd, 'bo');
hold on;
plot(norm_ssd_pixel, 'r*');

%% use the first image I1 as the template, plot SSD of other images against
% it
init_feat = y(:,1);
ssd = zeros(24, 1);
for i = 2:24
   feat_diff = y(:,i) - init_feat;
   ssd(i) = sum(feat_diff.^2);
end
ssd(1)=sum((eigen_patch'*black_img-init_feat).^2);
figure(1);subplot(2,1,1);
plot(ssd);
title('Sanity check SSD w/o per image normalization');
xlabel('frames');ylabel('SSD on coefficients');

%% Compare with pure SSD
init_pixel = centered_img(:,1);
init_pixel = init_pixel-min(init_pixel)/(max(init_pixel)-min(init_pixel));

ssd_pixel = zeros(24, 1);
for i = 2:24
   pixel_diff = centered_img(:,i) - init_pixel;
   ssd_pixel(i) = sum(pixel_diff.^2);
end
ssd_pixel(1)=sum((bad_img-init_pixel).^2);
subplot(2,1,2);
plot(ssd_pixel);
title('Sanity check SSD on pixels w/o per image normalization');
xlabel('frames');ylabel('SSD on pixels');

%% Normalize each feature vector
normalized_y = y;
for i = 1:24
    normalized_y(:,i) = y(:,i)./norm(y(:,i));
end
init_feat_unit = normalized_y(:,1);
ssd_unit = zeros(24, 1);
for i = 2:24
   feat_diff_unit = normalized_y(:,i) - init_feat_unit;
   ssd_unit(i) = sum(feat_diff_unit.^2);
end
bad_feat_unit = eigen_patch'*black_img;
bad_feat_unit = bad_feat_unit / norm(bad_feat_unit);
ssd_unit(1)=sum((bad_feat_unit-init_feat_unit).^2);
subplot(2,1,2);
plot(ssd_unit);
title('Sanity check SSD with unit feature');
xlabel('frames');ylabel('SSD on unit coefficients');

%% Per image standardization
stand_imgs = zeros(2500,24);
for i=1:24
     tmp = img(:,i) - mean(img(:,i));
     stand_imgs(:,i) = tmp/std(tmp);
end
stand_mean_img = mean(stand_imgs, 2);
stand_centered_img = stand_imgs - stand_mean_img * ones(1, size(stand_imgs, 2));
stand_A = stand_centered_img' * stand_centered_img / (size(stand_centered_img,2)-1);
[stand_V,stand_D] = eig(stand_A);
stand_eigen_patch = stand_centered_img * stand_V(:,end-9:end);
% project image onto eigen_patch
stand_coeff = stand_eigen_patch' * stand_centered_img;
stand_y = zeros(10, 24);
for i=1:24
    stand_y(:,i) = stand_eigen_patch' * stand_centered_img(:,i);
end

stand_init_feat = stand_y(:,1);
stand_ssd = zeros(24, 1);
for i = 2:24
   stand_feat_diff = stand_y(:,i) - stand_init_feat;
   stand_ssd(i) = sum(stand_feat_diff.^2);
end
stand_black_img = black_img - mean(black_img)/std(black_img);
stand_ssd(1)=sum((stand_eigen_patch'*stand_black_img-stand_init_feat).^2);
figure(1);subplot(2,1,2);plot(stand_ssd);
title('Sanity check SSD with per image normalization');
xlabel('frames');ylabel('SSD on coefficients');


%B1 = eigen_patch' * (img(:,1));
% Visualize basis images
% figure;
% for i=1:5
%     subplot(5,1,i);
%     imshow(reshape(eigen_patch(:,i),50,50), []);
% end


% img = imgs';
% [coef, score, latent, ~,explained] = pca(img);
% mean_img = mean(img, 2);
% centered_img = img - mean_img * ones(1, size(img, 2));
% basis_img = centered_img * coef;
% basis_img2 = score;
% %basis_img = centered_img * coef;
% % manual reconstr
% figure(2);
% for i=1:10
%     subplot(2,10,i);
%     imshow(reshape(basis_img(:,i), 50,50),[]);
%     pause;
% end
% % returned from matlab pca score
% figure(2);
% for i=1:10
%     subplot(2,10,i+10);
%     imshow(reshape(basis_img2(:,i), 50,50),[]);
% end
% 
% figure(3);
% img_recon = score * coef';
% for i=1:rows
%     subplot(rows,1,i);
%     imshow(reshape(img_recon(:,1),50,50), []);
% end
% 
% basis = basis_img(:,1:10);
% feat = basis' * img(:,1);
% recont = basis * feat;
% figure;
% imshow(reshape(recont,50,50), []);