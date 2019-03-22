###################
library(rgl)

plot.in.3d <- function(x,y,z, color, new.plot=TRUE, lim = c(-1,1))
{
  if (new.plot == TRUE) 
  {
    rgl.open()
  }
  if (is.null(color))
  {
    color <- as.hexmode(floor(seq(from = 0, to = 254,by = (255/(length(x) / 3)))))
    cc <- paste("#", color, "0000", sep = "")
    color <- c(cc, paste("#ff", color, "00", sep = ""),
               paste("#ff", rev(color), "ff", sep = "")
               ,"#ff00ff")
    
    
    plot3d(x, y, z, 
           xlab = "X", ylab = "Y", zlab = "Z",
           col = color, add=(!new.plot),
           xlim = lim,ylim = lim,zlim = lim)
  } else
  {
    if (class(color) == 'hexmode')
    {
      plot3d(x, y, z, 
             xlab = "X", ylab = "Y", zlab = "Z",
             col = paste("#", color, color, "00", sep = ""), add=(!new.plot),
             xlim = lim,ylim = lim,zlim = lim)
    } else {
    plot3d(x, y, z, 
           xlab = "X", ylab = "Y", zlab = "Z",
           col = 'green', add=(!new.plot),
           xlim = lim,ylim = lim,zlim = lim)
    }
  }
  return (color)
}

mulleft <- function(this, other)
{
  newX = other[4] * this[1] + other[1] * this[4] + other[2] * this[3] - other[3] * this[2]
  newY = other[4] * this[2] + other[2] * this[4] + other[3] * this[1] - other[1] * this[3]
  newZ = other[4] * this[3] + other[3] * this[4] + other[1] * this[2] - other[2] * this[1]
  newW = other[4] * this[4] - other[1] * this[1] - other[2] * this[2] - other[3] * this[3]
  return (c(newX,newY,newZ,newW))
}

transform <- function(x,y,z,w,vx,vy,vz)
{
  tmp2 = c(-x, -y, -z, w)
  tmp1 = c(vx, vy, vz, 0)
  rr = mulleft(mulleft(tmp2, tmp1),c(x,y,z,w))
  return (c(rr[1],rr[2],rr[3]))
}

read.head.rotation <- function(path.to.file)
{
  #path.to.file <- "e:\\Projects\\HeadMotion\\1551010782926.log"
  df <- read.csv(path.to.file, sep = ";", header = FALSE, stringsAsFactors = FALSE)
  df[df$V1 == 'q',3]
  library(RSpincalc)
  dfq <- df[df$V1 == 'q',3]
  xx <- list()
  yy <- list()
  zz <- list()
  for (a in 1:length(dfq))
  {
    quat <- as.numeric(unlist(strsplit(dfq[a], ",")))
    #Q <- c(quat[4],quat[3],quat[2],quat[1])
    #RightHandedToLeftHandedQuaternion
    Q <- c(quat[1],quat[2],quat[3],quat[4])
    
    quatbase <- as.numeric(unlist(strsplit(dfq[1], ",")))
    Qbase <- c(quatbase[1],quatbase[2],quatbase[3],quatbase[4])
    #RightHandedToLeftHandedQuaternion
    #Qbase <- c(-quatbase[4],-quatbase[3],-quatbase[2],quatbase[1])
    
    #x z
    hfold = transform(quatbase[1],quatbase[2],quatbase[3],quatbase[4],1,0,0)
    hfnew = transform(quat[1],quat[2],quat[3],quat[4],0,0,1)
    roty <- acos(sum(hfold * hfnew)) * 180 / pi
    
    #z y
    hfold = transform(quatbase[1],quatbase[2],quatbase[3],quatbase[4],0,0,1)
    hfnew = transform(quat[1],quat[2],quat[3],quat[4],0,1,0)
    rotx <- acos(sum(hfold * hfnew)) * 180 / pi
    
    #x y
    hfold = transform(quatbase[1],quatbase[2],quatbase[3],quatbase[4],1,0,0)
    hfnew = transform(quat[1],quat[2],quat[3],quat[4],0,1,0)
    rotz <- acos(sum(hfold * hfnew)) * 180 / pi
    
    xx[[a]] <- rotx
    yy[[a]] <- roty
    zz[[a]] <- rotz
  }
  return (data.frame(pitch = (180 - unlist(xx) - 90), yaw = unlist(yy) - 90, roll = (180 - unlist(zz)- 90)))
}

read.log.quaternion <- function(path.to.file)
{
  df <- read.csv(path.to.file, sep = ";", header = FALSE, stringsAsFactors = FALSE)
  df[df$V1 == 'q',3]
  library(RSpincalc)
  dfq <- df[df$V1 == 'q',3]
  xx <- list()
  yy <- list()
  zz <- list()
  
  #V <- c(-1,0,0)
  V <- c(0,1,0)
  for (a in 1:length(dfq))
  {
    #x y z w
    #y,-z,-w,-x
    quat <- as.numeric(unlist(strsplit(dfq[a], ",")))
    #Q <- c(quat[4],quat[3],quat[2],quat[1])
    #RightHandedToLeftHandedQuaternion
    Q <- c(quat[1],quat[2],quat[3],quat[4])
    Vrot <- transform(quat[1],quat[2],quat[3],quat[4],0,0,1)
    xx[[a]] <- Vrot[1]
    yy[[a]] <- Vrot[2]
    zz[[a]] <- Vrot[3]
  }
  return (data.frame(x = unlist(xx), y = unlist(yy), z = unlist(zz)))
}
#dó³, góra, lewo, prawo, zgodnie z ruchem
filePath <- "e:\\aa\\1551035595184.log"
#filePath <- "e:\\Projects\\HeadMotion\\1551030542974.log"
#dó³, góra, lewo, prawo, zgodnie z ruchem
#filePath <- "e:\\Publikacje\\RobotVR\\R\\1552601786581.log"


df.helper <- read.csv(filePath, sep = ";", header = FALSE, stringsAsFactors = FALSE)
time.helper <- df.helper[df.helper$V1 == 'q',2]
for (a in length(time.helper):1)
{
  time.helper[a] <- time.helper[a] - time.helper[1]
}

#góra dó³ lewo prawo 1551030542974.log
#dó³ góra lewo prawo zgodnie 1551030542974.log

df.motion <- read.log.quaternion(filePath)
df.rotation <- read.head.rotation(filePath)

plot(x = time.helper / 1000, df.rotation$pitch, xlab = "Time [s]", ylab = "Angle [degress]", main = "Head rotation (vertical)")
lines(x=c(0,time.helper[length(time.helper)]), y = c(0,0), col = "blue")

plot(x = time.helper / 1000, df.rotation$yaw, xlab = "Time [s]", ylab = "Angle [degress]", main = "Head rotation (horizontal)")
lines(x=c(0,time.helper[length(time.helper)]), y = c(0,0), col = "blue")

plot(x = time.helper / 1000, df.rotation$roll, xlab = "Time [s]", ylab = "Angle [degress]", main = "Head rotation (bending)")
lines(x=c(0,time.helper[length(time.helper)]), y = c(0,0), col = "blue")


color = plot.in.3d(df.motion[,1],df.motion[,2],df.motion[,3], color = NULL)
color = plot.in.3d(df.rotation$yaw,df.rotation$pitch,0, color = NULL, lim = NULL)



color = plot.in.3d(seq(0,length(df.rotation$yaw)),df.rotation$yaw,0, color = NULL, lim = NULL)
color = plot.in.3d(seq(0,length(df.rotation$pitch)),df.rotation$pitch,0, color = NULL, lim = NULL)
color = plot.in.3d(seq(0,length(df.rotation$roll)),df.rotation$roll,0, color = NULL, lim = NULL)

color = plot.in.3d(df.rotation$yaw,df.rotation$pitch,df.rotation$roll, color = NULL, lim = NULL)


library(sphereplot)
rgl.sphgrid(radaxis=FALSE, longtype = 'D', col.long = "black", col.lat = "blue", add = TRUE)
#rgl.sphpoints(df.rotation$yaw,df.rotation$pitch,rep(1,length(df.rotation$yaw)),deg=TRUE,col='red')
#points3d(df.motion[,1],df.motion[,2],df.motion[,3], size=4)
#dfff <- sph2car(df.rotation$yaw,df.rotation$pitch)
dfff <- sph2car(long = df.rotation$pitch,lat =  df.rotation$yaw)


rgl.open()
bg3d("#777777")
#spheres3d(0,0,0,lit=FALSE,color="white", alpha=0.9)
#spheres3d(0,0,0,radius=1.01,lit=FALSE,color="black",front="lines", alpha=0.05)
#set.seed(101)
#n <- 50
#theta <- runif(n,0,2*pi)
#u <- runif(n,-1,1)
#x <- sqrt(1-u^2)*cos(theta)
#y <- sqrt(1-u^2)*sin(theta)
#z <- u
#spheres3d(x,y,z,col="red",radius=0.02)
rgl.spheres(0, 0, 0, radius = 0.99, col = 'white', alpha = 0.6, back = 'lines')
rgl.spheres(dfff[,1],dfff[,2],dfff[,3], r = 0.025,col=color)
#rgl.points(dfff[,1],dfff[,2],dfff[,3], size=4,col='red')


#rgl.postscript( filename = "e:\\Projects\\HeadMotion\\a.pdf", fmt = "pdf", drawText = TRUE)

#OKOKOK

library(sphereplot)
rgl.open()
rgl.sphgrid(radaxis=FALSE, longtype = 'D', col.long = "black", col.lat = "blue", add = TRUE)
#rgl.sphpoints(df.rotation$yaw,df.rotation$pitch,rep(1,length(df.rotation$yaw)),deg=TRUE,col='red')
#points3d(df.motion[,1],df.motion[,2],df.motion[,3], size=4)
#dfff <- sph2car(df.rotation$yaw,df.rotation$pitch)
dfff <- sph2car(long = df.rotation$yaw,lat =  df.rotation$pitch)
rgl.spheres(dfff[,1],dfff[,2],dfff[,3], r = 0.025,col=color)


U <- par3d("userMatrix")
for (theta in seq(0, pi, len=100)) {
  par3d(userMatrix = rotate3d(U, theta, 0,0,1)) # Rotate about model's
  Sys.sleep(0.1)
} 
