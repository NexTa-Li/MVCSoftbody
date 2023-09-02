import controller.SoftBodyController;
import model.SoftBodyModel;
import view.SoftBodyView;

public class Driver {
    SoftBodyModel model;
    SoftBodyController controller;
    SoftBodyView view;

    Driver() {
        model = new SoftBodyModel();
        controller = new SoftBodyController(model);
        view = new SoftBodyView(model, controller);
    }

    public static void main(String[] args) {
        Driver driver = new Driver();
    }
}
