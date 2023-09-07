package controller;

import model.SoftBodyModel;
import model.geometry.Point2D;
import model.softbody.SoftBodyUtil;

import java.awt.event.*;

/**
 * Deciding between having a view in the controller and updating from here or
 * not.
 * 
 * I *think* in this case for optimization its better to just pass a ReadOnly
 * list of the softbodies
 * to the view in the very beginning, that way it can display them without
 * having to wait for the
 * controller to update them.
 */
public class SoftBodyController extends KeyAdapter implements MouseListener,
        MouseMotionListener, MouseWheelListener, ControllerConfig {

    SoftBodyModel model;

    // mouse location
    static int mouseX, mouseY;

    // input flags
    static boolean keyUp = false;
    static boolean keyDown = false;
    static boolean keyLeft = false;
    static boolean keyRight = false;
    static boolean keyA = false;
    static boolean keyD = false;

    static boolean mousePressed = false;
    static boolean mouseReleased = false;
    static boolean mouseInPanel = false;

    static int selectedSoftbody = 0; // index of the selected softbody

    public SoftBodyController(SoftBodyModel model) {
        this.model = model;
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        mouseX = e.getX();
        mouseY = e.getY();
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        mouseX = e.getX();
        mouseY = e.getY();
    }

    @Override
    public void mouseClicked(MouseEvent e) {
    }

    // temporarily commented out
    @Override
    public void mousePressed(MouseEvent e) {

        if (mouseInPanel) {

            mousePressed = true;
            mouseX = e.getX();
            mouseY = e.getY();
            // System.out.println("Mouse pressed at: " + mouseX + ", " + mouseY);
            System.out.println(".addPoint(" + mouseX + ", " + mouseY + ");");

            for (int i = 0; i < model.getSoftBodies().size(); i++) {
                if (SoftBodyUtil.checkCollision(new Point2D(mouseX, mouseY), model.getSoftBodies().get(i))) {
                    System.out.println("Soft body " + i + " was clicked on");
                    // update display info

                    selectedSoftbody = i;
                    break;
                }
            }
        }
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        mousePressed = false;
    }

    // only handle mouse events if the mouse is in the panel
    @Override
    public void mouseEntered(MouseEvent e) {
        mouseInPanel = true;
    }

    @Override
    public void mouseExited(MouseEvent e) {
        mouseInPanel = false;
    }

    @Override
    public void keyPressed(KeyEvent e) {

        // for (SoftBody softBody : softBodies) {
        // softBody.keyPressed(e);
        // }

        model.getSoftBodies().get(selectedSoftbody).keyPressed(e);

        if (e.getKeyCode() == KeyEvent.VK_UP)
            keyUp = true;
        if (e.getKeyCode() == KeyEvent.VK_DOWN)
            keyDown = true;
        if (e.getKeyCode() == KeyEvent.VK_LEFT)
            keyLeft = true;
        if (e.getKeyCode() == KeyEvent.VK_RIGHT)
            keyRight = true;
        if (e.getKeyCode() == KeyEvent.VK_A)
            keyA = true;
        if (e.getKeyCode() == KeyEvent.VK_D)
            keyD = true;

        // used to test frame by frame
        if (e.getKeyCode() == KeyEvent.VK_SPACE)
            model.idle();
    }

    @Override
    public void keyReleased(KeyEvent e) {

        // for (SoftBody softBody : softBodies) {
        // softBody.keyReleased(e);
        // }
        model.getSoftBodies().get(selectedSoftbody).keyReleased(e);

        if (e.getKeyCode() == KeyEvent.VK_UP)
            keyUp = false;
        if (e.getKeyCode() == KeyEvent.VK_DOWN)
            keyDown = false;
        if (e.getKeyCode() == KeyEvent.VK_LEFT)
            keyLeft = false;
        if (e.getKeyCode() == KeyEvent.VK_RIGHT)
            keyRight = false;
        if (e.getKeyCode() == KeyEvent.VK_A)
            keyA = false;
        if (e.getKeyCode() == KeyEvent.VK_D)
            keyD = false;

    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        // int rotation = e.getWheelRotation();

        // if (rotation > 0) {
        // SoftBodyPanel.selectedPoint = (SoftBodyPanel.selectedPoint + 1)
        // % (int) softBodies.get(SoftBodyPanel.selectedSoftbodyIndex).getNumPoints();

        // } else {
        // SoftBodyPanel.selectedPoint = (SoftBodyPanel.selectedPoint - 1
        // + (int) softBodies.get(SoftBodyPanel.selectedSoftbodyIndex).getNumPoints())
        // % (int) softBodies.get(SoftBodyPanel.selectedSoftbodyIndex).getNumPoints();
        // }
    }

    public int getSelectedSoftbodyIndex() {
        return selectedSoftbody;
    }
}
