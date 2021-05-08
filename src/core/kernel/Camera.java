package core.kernel;

import core.math.Matrix4f;
import core.math.Vec3f;
import core.math.Vec4f;
import core.physics.Physics;
import core.utils.Constants;
import dc.utils.Frustum;
import dc.utils.Ray;

import static org.lwjgl.glfw.GLFW.*;

public class Camera {
	
	private static Camera instance = null;
	private static final int BOUNDED_VELOCITY = 15;

	private final Vec3f yAxis = new Vec3f(0,1,0);
	private final Vec3f speed = new Vec3f(0);

	private Vec3f position;
	private Vec3f previousPosition;
	private Vec3f forward;
	private Vec3f previousForward;
	private Vec3f up;
	private float movAmt = 0.1f;
	private float rotAmt = 0.8f;
	private Matrix4f viewMatrix;
	private Matrix4f projectionMatrix;
	private Matrix4f viewProjectionMatrix;
	private boolean cameraMoved;
	private boolean cameraRotated;
	
	private float width;
	private float height;
	private float fovY;

	private float rotYstride;
	private float rotYamt;
	private float rotYcounter;
	private boolean rotYInitiated = false;
	private float rotXstride;
	private float rotXamt;
	private float rotXcounter;
	private boolean rotXInitiated = false;
	private float mouseSensitivity = 0.8f;

	private Vec3f velocity = new Vec3f();
	private Physics physics;
	private final Ray ray = new Ray(new Vec3f(), new Vec3f());
	  
	public static Camera getInstance() 
	{
	    if(instance == null) 
	    {
	    	instance = new Camera();
	    }
	      return instance;
	}
	
	protected Camera() {
		this(new Vec3f(906,-1509,-2694),  new Vec3f(1,0,0).normalize(), new Vec3f(0,1,0));
		setProjection(70, Window.getInstance().getWidth(), Window.getInstance().getHeight());
		viewMatrix = new Matrix4f().View(forward, up).mul(new Matrix4f().Translation(position.mul(-1)));
		viewProjectionMatrix = new Matrix4f().Zero();
		viewProjectionMatrix = projectionMatrix.mul(viewMatrix);
		Frustum.getFrustum().calculateFrustum(viewProjectionMatrix);
	}
	
	private Camera(Vec3f position, Vec3f forward, Vec3f up)
	{
		setPosition(position);
		setForward(forward);
		setUp(up);
		up.normalize();
		forward.normalize();
	}
	
	public void update()
	{
		setPreviousPosition(new Vec3f(position));
		setPreviousForward(new Vec3f(forward));
		cameraMoved = false;
		cameraRotated = false;
		

		movAmt += (0.04f * Input.getInstance().getScrollOffset());
		movAmt = Math.max(0.02f, movAmt);
		//movAmt = 1;
		
		if(Input.getInstance().isKeyHold(GLFW_KEY_W)) {
			speed.Z = movAmt;
			move(getForward(), movAmt);
		}
		if(Input.getInstance().isKeyHold(GLFW_KEY_S)) {
			speed.Z = -movAmt;
			move(getForward(), -movAmt);
		}
		if(Input.getInstance().isKeyHold(GLFW_KEY_A)) {
			speed.X = -movAmt;
			move(getLeft(), movAmt);
		}
		if(Input.getInstance().isKeyHold(GLFW_KEY_D)) {
			speed.X = movAmt;
			move(getRight(), movAmt);
		}

		if(Input.getInstance().isKeyReleased(GLFW_KEY_W) || Input.getInstance().isKeyReleased(GLFW_KEY_S)) {
			speed.Z = 0.f;
		}
		if(Input.getInstance().isKeyReleased(GLFW_KEY_A) || Input.getInstance().isKeyReleased(GLFW_KEY_D)) {
			speed.X = 0.f;
		}
				
		if(Input.getInstance().isKeyHold(GLFW_KEY_UP))
			rotateX(-rotAmt/8f);
		if(Input.getInstance().isKeyHold(GLFW_KEY_DOWN))
			rotateX(rotAmt/8f);
		if(Input.getInstance().isKeyHold(GLFW_KEY_LEFT))
			rotateY(-rotAmt/8f);
		if(Input.getInstance().isKeyHold(GLFW_KEY_RIGHT))
			rotateY(rotAmt/8f);

		if (Input.getInstance().isKeyHold(GLFW_KEY_SPACE)) {
			velocity.set(0, 0, 0);
		}
		
		// free mouse rotation
		if(Input.getInstance().isButtonHolding(2))
		{
			float dy = Input.getInstance().getLockedCursorPosition().getY() - Input.getInstance().getCursorPosition().getY();
			float dx = Input.getInstance().getLockedCursorPosition().getX() - Input.getInstance().getCursorPosition().getX();
			
			// y-axxis rotation
			
			if (dy != 0){
				rotYstride = Math.abs(dy * 0.01f);
				rotYamt = -dy;
				rotYcounter = 0;
				rotYInitiated = true;
			}
			
			if (rotYInitiated ){
				
				// up-rotation
				if (rotYamt < 0){
					if (rotYcounter > rotYamt){
						rotateX(-rotYstride * mouseSensitivity);
						rotYcounter -= rotYstride;
						rotYstride *= 0.98;
					}
					else rotYInitiated = false;
				}
				// down-rotation
				else if (rotYamt > 0){
					if (rotYcounter < rotYamt){
						rotateX(rotYstride * mouseSensitivity);
						rotYcounter += rotYstride;
						rotYstride *= 0.98;
					}
					else rotYInitiated = false;
				}
			}
			
			// x-axxis rotation
			if (dx != 0){
				rotXstride = Math.abs(dx * 0.01f);
				rotXamt = dx;
				rotXcounter = 0;
				rotXInitiated = true;
			}
			
			if (rotXInitiated){
				
				// up-rotation
				if (rotXamt < 0){
					if (rotXcounter > rotXamt){
						rotateY(rotXstride * mouseSensitivity);
						rotXcounter -= rotXstride;
						rotXstride *= 0.96;
					}
					else rotXInitiated = false;
				}
				// down-rotation
				else if (rotXamt > 0){
					if (rotXcounter < rotXamt){
						rotateY(-rotXstride * mouseSensitivity);
						rotXcounter += rotXstride;
						rotXstride *= 0.96;
					}
					else rotXInitiated = false;
				}
			}
			
			glfwSetCursorPos(Window.getInstance().getWindow(),
					 Input.getInstance().getLockedCursorPosition().getX(),
					 Input.getInstance().getLockedCursorPosition().getY());
		}
		
		if (!position.equals(previousPosition)){
			cameraMoved = true;	
		}
		
		if (!forward.equals(previousForward)){
			cameraRotated = true;
		}

		viewMatrix = new Matrix4f().View(forward, up).mul(new Matrix4f().Translation(position.mul(-1)));
		viewProjectionMatrix = projectionMatrix.mul(viewMatrix);
		processPhysics(physics, speed);
//		Matrix4f worldView1 = glmLookAt(position.add(forward), position, up);
//		Matrix4f m = projectionMatrix.mul(worldView1);
		Frustum.getFrustum().calculateFrustum(viewProjectionMatrix);
	}

	private void processPhysics(Physics physics, Vec3f speed) {
		if(physics!=null) {
			position = physics.Physics_GetPlayerPosition();
			velocity = velocity.add(forward.mul(speed.Z));
			velocity = velocity.add(up.mul(speed.Y));
			velocity = velocity.add(getRight().mul(speed.X));

			if(velocity.X > BOUNDED_VELOCITY){
				velocity.X = BOUNDED_VELOCITY;
			}
			if(velocity.X < -BOUNDED_VELOCITY){
				velocity.X = -BOUNDED_VELOCITY;
			}

			if(velocity.Y > BOUNDED_VELOCITY){
				velocity.Y = BOUNDED_VELOCITY;
			}
			if(velocity.Y < -BOUNDED_VELOCITY){
				velocity.Y = -BOUNDED_VELOCITY;
			}

			if(velocity.Z > BOUNDED_VELOCITY){
				velocity.Z = BOUNDED_VELOCITY;
			}
			if(velocity.Z < -BOUNDED_VELOCITY){
				velocity.Z = -BOUNDED_VELOCITY;
			}
			physics.Physics_SetPlayerVelocity(velocity);
		}
	}

	public void setPhysics(Physics physics){
		this.physics = physics;
	}
	
	public void move(Vec3f dir, float amount)
	{
		Vec3f newPos = position.add(dir.mul(amount));
		setPosition(newPos);
	}
	
	public void rotateY(float angle)
	{
		Vec3f hAxis = yAxis.cross(forward).normalize();
		
		forward.rotate(angle, yAxis).normalize();
		
		up = forward.cross(hAxis).normalize();
	}
	
	public void rotateX(float angle)
	{
		Vec3f hAxis = yAxis.cross(forward).normalize();

		forward.rotate(angle, hAxis).normalize();
		
		up = forward.cross(hAxis).normalize();
	}
	
	public Vec3f getLeft()
	{
		Vec3f left = forward.cross(up);
		left.normalize();
		return left;
	}
	
	public Vec3f getRight()
	{
		Vec3f right = up.cross(forward);
		right.normalize();
		return right;
	}

	public Matrix4f getProjectionMatrix() {
		return projectionMatrix;
	}
	
	public  void setProjection(float fovY, float width, float height)
	{
		this.fovY = fovY;
		this.width = width;
		this.height = height;
		
		this.projectionMatrix = new Matrix4f().PerspectiveProjection(fovY, width, height, Constants.ZNEAR, Constants.ZFAR);
	}

	public Matrix4f getViewMatrix() {
		return viewMatrix;
	}

	public Vec3f getPosition() {
		return position;
	}

	public void setPosition(Vec3f position) {
		this.position = position;
	}

	public Vec3f getForward() {
		return forward;
	}

	public void setForward(Vec3f forward) {
		this.forward = forward;
	}

	public Vec3f getUp() {
		return up;
	}

	public void setUp(Vec3f up) {
		this.up = up;
	}
	
	public float getFovY(){
		return this.fovY;
	}
	
	public float getWidth(){
		return this.width;
	}

	public float getHeight(){
		return this.height;
	}
	
	public Matrix4f getViewProjectionMatrix() {
		return viewProjectionMatrix;
	}

	public boolean isCameraMoved() {
		return cameraMoved;
	}

	public boolean isCameraRotated() {
		return cameraRotated;
	}
	
	public Vec3f getPreviousPosition() {
		return previousPosition;
	}

	public void setPreviousPosition(Vec3f previousPosition) {
		this.previousPosition = previousPosition;
	}
	
	public Vec3f getPreviousForward() {
		return previousForward;
	}

	private void setPreviousForward(Vec3f previousForward) {
		this.previousForward = previousForward;
	}

	public Vec3f unproject (Vec3f screenCoords, float viewportX, float viewportY, float viewportWidth, float viewportHeight) {
		float x = screenCoords.X, y = screenCoords.Y;
		x = x - viewportX;
		y = Window.getInstance().getHeight() - y;
		y = y - viewportY;

		Vec3f tmpVec = new Vec3f();
		tmpVec.X = (2 * x) / viewportWidth - 1;
		tmpVec.Y = (2 * y) / viewportHeight - 1;
		tmpVec.Z = 2 * screenCoords.Z - 1;
		Matrix4f invProjectionView = viewProjectionMatrix.invert();
		return tmpVec.project(invProjectionView);
	}

	public Ray getCrossHairRay(float rayLength) {
		Vec3f start = getPosition();
		Vec3f camRayEnd = getForward().scaleAdd(rayLength, start);
		this.ray.origin.set(start);
		this.ray.direction.set(camRayEnd);
		return ray;
	}

	private Ray getPickRay(float screenX, float screenY, float viewportX, float viewportY, float viewportWidth, float viewportHeight) {
		Vec3f origin = new Vec3f(screenX, screenY, 0);
		Vec3f direction = new Vec3f(screenX, screenY, 1);
		Vec3f worldSpaceOrigin = unproject(origin, viewportX, viewportY, viewportWidth, viewportHeight);
		Vec3f worldSpaceDirection = unproject(direction, viewportX, viewportY, viewportWidth, viewportHeight);
		worldSpaceDirection = worldSpaceDirection.sub(worldSpaceOrigin).normalize();

		this.ray.origin.set(worldSpaceOrigin);
		this.ray.direction.set(worldSpaceDirection);
		return ray;
	}

	public Ray getMousePickRay(float screenX, float screenY) {
		return getPickRay(screenX, screenY, 0, 0, Window.getInstance().getWidth(), Window.getInstance().getHeight());
	}



	public static float clamp(float s, float min, float max) {
		return Math.min(Math.max(s, min), max);
	}

	float		rotateX_ = 0.f;
	float		rotateY_ = 180.f;
	Vec3f		forward_  = new Vec3f(0.f, 0.f, 1.f );
	Vec3f		right_  = new Vec3f(1.f, 0.f, 0.f );
	Vec3f		up_  = new Vec3f( 0.f, 1.f, 0.f);

	void update(Vec3f speed, float rotateX, float rotateY) {
		rotateX_ += rotateX;
		rotateX_ = clamp(rotateX_, -85.f, 85.f);

		rotateY_ += rotateY;

		Matrix4f pitchMatrix = Matrix4f.rotate(new Matrix4f().Identity(), rotateX_, new Vec3f(1, 0, 0));
		Matrix4f yawMatrix = Matrix4f.rotate(new Matrix4f().Identity(), rotateY_, new Vec3f(0, 1, 0));

		Vec4f forward = new Vec4f(0.f, 0.f, 1.f, 0.f);
		forward = pitchMatrix.mul(forward);
		forward = yawMatrix.mul(forward);
		forward_.set(forward);

		Vec4f right = new Vec4f(1.f, 0.f, 0.f, 0.f);
		right = pitchMatrix.mul(right);
		right = yawMatrix.mul(right);
		right_.set(right);

		Vec4f up = new Vec4f(0.f, 1.f, 0.f, 0.f);
		up = pitchMatrix.mul(up);
		up = yawMatrix.mul(up);
		up_.set(up);

		processPhysics(physics, speed);
	}
}