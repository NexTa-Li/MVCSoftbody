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
    static boolean mousePressed = false;
    static boolean mouseReleased = false;
    static boolean mouseInPanel = false;

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

                    model.selectedSoftbodyIndex = i;
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

        if (e.getKeyCode() == ToggleSoftBodyInfo)
            model.drawBodyInfo = !model.drawBodyInfo;

        if (e.getKeyCode() == KeyEvent.VK_UP)
            model.getSoftBodies().get(model.getId()).keyUp = true;
        if (e.getKeyCode() == KeyEvent.VK_DOWN)
            model.getSoftBodies().get(model.getId()).keyDown = true;
        if (e.getKeyCode() == KeyEvent.VK_LEFT)
            model.getSoftBodies().get(model.getId()).keyLeft = true;
        if (e.getKeyCode() == KeyEvent.VK_RIGHT)
            model.getSoftBodies().get(model.getId()).keyRight = true;
        if (e.getKeyCode() == KeyEvent.VK_PLUS || e.getKeyCode() == KeyEvent.VK_EQUALS)
            model.getSoftBodies().get(model.getId()).increase = true;
        if (e.getKeyCode() == KeyEvent.VK_MINUS || e.getKeyCode() == KeyEvent.VK_UNDERSCORE)
            model.getSoftBodies().get(model.getId()).decrease = true;
        if (e.getKeyCode() == KeyEvent.VK_7) {
            System.out.println("Pressure Change");
            model.getSoftBodies().get(model.getId()).resetChangeVars();
            model.getSoftBodies().get(model.getId()).pressureChange = true;
        }
        if (e.getKeyCode() == KeyEvent.VK_8) {
            System.out.println("Mass Change");
            model.getSoftBodies().get(model.getId()).resetChangeVars();
            model.getSoftBodies().get(model.getId()).massChange = true;
        }
        if (e.getKeyCode() == KeyEvent.VK_9) {
            System.out.println("Spring Constant Change");
            model.getSoftBodies().get(model.getId()).resetChangeVars();
            model.getSoftBodies().get(model.getId()).springConstantChange = true;
        }
        if (e.getKeyCode() == KeyEvent.VK_0) {
            System.out.println("Spring Length Change");
            model.getSoftBodies().get(model.getId()).resetChangeVars();
            model.getSoftBodies().get(model.getId()).springLengthChange = true;
        }

        // used to test frame by frame
        if (e.getKeyCode() == KeyEvent.VK_SPACE)
            model.idle();
    }

    @Override
    public void keyReleased(KeyEvent e) {

        if (e.getKeyCode() == KeyEvent.VK_UP)
            model.getSoftBodies().get(model.getId()).keyUp = false;
        if (e.getKeyCode() == KeyEvent.VK_DOWN)
            model.getSoftBodies().get(model.getId()).keyDown = false;
        if (e.getKeyCode() == KeyEvent.VK_LEFT)
            model.getSoftBodies().get(model.getId()).keyLeft = false;
        if (e.getKeyCode() == KeyEvent.VK_RIGHT)
            model.getSoftBodies().get(model.getId()).keyRight = false;
        if (e.getKeyCode() == KeyEvent.VK_PLUS || e.getKeyCode() == KeyEvent.VK_EQUALS)
            model.getSoftBodies().get(model.getId()).increase = false;
        if (e.getKeyCode() == KeyEvent.VK_MINUS || e.getKeyCode() == KeyEvent.VK_UNDERSCORE)
            model.getSoftBodies().get(model.getId()).decrease = false;
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
}
